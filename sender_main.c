/*!
    \file    app.c
    \brief   USB CDC RGB565 image + MCU-side center marker + single AI box + gimbal PID + lightweight tracking

    \version 2025-02-19, V2.1.0, demo for GD32H7xx

    Current baseline:
        MCU-side drawing version.

    Current display behavior:
        1. OV5640 image is sent to PC / IMX6ULL through USB CDC.
        2. MCU draws:
              - green cross at image center
              - only one selected red AI detection box
        3. MCU keeps one tracked AI target for:
              - gimbal PID control
              - USB detection packet
              - debug best-target variables
        4. USB image packet:
              type   = 0x01
              format = 0x01 RGB565
        5. USB detection packet:
              type   = 0x02
              format = 0x02 DET_V2
              payload contains one selected target and X servo delta angle.

    Gimbal behavior:
        1. PB3  = X axis servo, horizontal direction.
        2. PA15 = Y axis servo, vertical direction.
        3. AI frame with tracked target:
              PID updates gimbal angle once.
        4. AI frame without tracked target:
              hold current position for short time.
              after continuous lost threshold, return slowly to 90-degree center.

    Lightweight tracking behavior:
        1. Candidate target:
              confidence must be greater than or equal to 0.60.
        2. Each AI frame:
              choose the highest-confidence valid target.
        3. Target center:
              smooth center when the selected target stays near previous center.
        4. No valid target:
              mark target lost and let gimbal lost-target policy handle it.
*/

#include "gd32h7xx.h"
#include "drv_usb_hw.h"
#include "cdc_acm_core.h"
#include "exmc_sdram.h"
#include "dci_ov5640.h"
#include "systick.h"
#include "ai_model.h"
#include "mg90s.h"

#include <stdint.h>
#include <string.h>

/* ============================================================
   Debug switch
   ============================================================ */

#define APP_DEBUG_ENABLE                         0U

#if APP_DEBUG_ENABLE
#define APP_DEBUG_CODE(code)                     do { code } while(0)
#else
#define APP_DEBUG_CODE(code)                     do { } while(0)
#endif

#ifndef MPU_ACCESS_BUFFERABLE
#define MPU_ACCESS_BUFFERABLE                    ((uint8_t)1U)
#endif

#ifndef MPU_ACCESS_CACHEABLE
#define MPU_ACCESS_CACHEABLE                     ((uint8_t)1U)
#endif

/* ============================================================
   App mode
   ============================================================ */

#define APP_TEST_USB_FAKE_IMAGE_ONLY             0U
#define APP_TEST_OV5640_FREEZE_FRAME_ONLY        0U

#define APP_AI_ENABLE                            1U

/* ============================================================
   Lightweight AI target tracking config
   ============================================================ */

#define AI_TRACK_ENABLE                          1U

#if APP_AI_ENABLE
#define APP_AI_CODE(code)                        do { code } while(0)
#else
#define APP_AI_CODE(code)                        do { } while(0)
#endif

#define APP_GIMBAL_PID_MASTER_ENABLE             1U

/*
    Run AI once every N camera frames.

    1 = run AI every frame.
    2 = run AI once every 2 frames.
*/
#define APP_AI_RUN_INTERVAL                      2U

#if ((APP_AI_ENABLE != 0U) && \
     (!APP_TEST_USB_FAKE_IMAGE_ONLY) && \
     (!APP_TEST_OV5640_FREEZE_FRAME_ONLY))
#define APP_ENABLE_AI_USB_RESULT                 1U
#else
#define APP_ENABLE_AI_USB_RESULT                 0U
#endif

#if ((APP_ENABLE_AI_USB_RESULT != 0U) && \
     (APP_GIMBAL_PID_MASTER_ENABLE != 0U))
#define APP_ENABLE_GIMBAL_PID                    1U
#else
#define APP_ENABLE_GIMBAL_PID                    0U
#endif

#define USB_TX_PACE_DELAY_MS                     0U
#define USB_BUSY_DROP_THRESHOLD                  200000U
#define USB_TX_BUFFER_COUNT                      2U

/*
    MCU-side marker style.
*/
#define USB_CENTER_CROSS_COLOR                   0x07E0U
#define USB_CENTER_CROSS_HALF_LEN                8
#define USB_CENTER_CROSS_THICKNESS               1

/*
    AI box drawing.

    Only one selected target is drawn to avoid multi-target interference.
*/
#define USB_AI_BOX_COLOR                         0xF800U
#define USB_AI_BOX_THICKNESS                     2
#define USB_AI_BOX_MIN_SIZE                      4
#define USB_AI_BOX_DRAW_MAX                      1U
#define USB_AI_BOX_DRAW_SELECTED_ONLY            1U

#define USB_DRAW_BEST_TARGET_CENTER              0U
#define USB_TARGET_CENTER_COLOR                  0xF800U
#define USB_TARGET_RECT_HALF_SIZE                3

#define APP_DCACHE_LINE_SIZE                     32U

/*
    256x256 image.
    80px allows moderate movement between AI frames while still avoiding
    immediate switching to another unrelated target.
*/
#define AI_TRACK_MATCH_MAX_DIST_PX               80.0f

/*
    Minimum confidence for a detection to be accepted as a tracking target.

    Detections with confidence lower than this value are ignored completely:
        - not used for initial lock
        - not used for tracking update
        - not drawn as selected box
        - not sent in USB detection packet
        - not used by gimbal PID
*/
#define AI_TRACK_CONF_MIN                        0.60f

/*
    Smaller alpha = smoother target center.
    Larger alpha  = faster response.

    0.40 keeps response fast enough while reducing box jitter.
*/
#define AI_TRACK_SMOOTH_ALPHA                    0.40f

#define AI_TRACK_REACQUIRE_LOST_COUNT            3U
#define AI_TRACK_LOST_COUNT_MAX                  100000U

/* ============================================================
   Gimbal PID config
   ============================================================ */

/*
    Servo assignment:
        PB3  -> TIMER1_CH1 -> X axis servo, S20F 270 degree
        PA15 -> TIMER1_CH0 -> Y axis servo, S20F 180 degree

    Current effective gimbal angle range confirmed by test:

        X axis PB3:
            45 degrees  = left
            90 degrees  = center
            135 degrees = right

        Y axis PA15:
            65 degrees  = up
            90 degrees  = center
            135 degrees = down

    Image direction and gimbal direction are consistent:

        target on left side   -> X angle decreases
        target on right side  -> X angle increases

        target on upper side  -> Y angle decreases
        target on lower side  -> Y angle increases
*/
#define GIMBAL_X_INIT_ANGLE                      90.0f
#define GIMBAL_Y_INIT_ANGLE                      90.0f

#define GIMBAL_X_ANGLE_MIN                       45.0f
#define GIMBAL_X_ANGLE_MAX                       135.0f

#define GIMBAL_Y_ANGLE_MIN                       65.0f
#define GIMBAL_Y_ANGLE_MAX                       135.0f

#define GIMBAL_X_DIR                             1.0f
#define GIMBAL_Y_DIR                             1.0f

/*
    PID parameters for 256x256 image.
*/
#define GIMBAL_X_KP                              0.110f
#define GIMBAL_X_KI                              0.0000f
#define GIMBAL_X_KD                              0.015f

#define GIMBAL_Y_KP                              0.075f
#define GIMBAL_Y_KI                              0.0000f
#define GIMBAL_Y_KD                              0.012f

/*
    Central dead zone.
*/
#define GIMBAL_DEADZONE_PX                       1.5f

#define GIMBAL_INTEGRAL_LIMIT                    500.0f
#define GIMBAL_MAX_STEP_DEG                      8.0f
#define GIMBAL_MIN_STEP_DEG                      0.35f

#define GIMBAL_LOST_HOLD_COUNT                   5U
#define GIMBAL_LOST_RETURN_COUNT                 1U
#define GIMBAL_LOST_COUNT_MAX                    100000U

#define GIMBAL_RETURN_STEP_DEG                   1.5f

usb_core_driver cdc_acm;

extern uint8_t cdc_acm_user_send(usb_core_driver *pudev, uint8_t *buf, uint32_t len);

/* ============================================================
   Debug variables
   ============================================================ */

#if APP_DEBUG_ENABLE

volatile uint32_t g_app_stage = 0U;

volatile uint32_t g_clk_after_systick = 0U;
volatile uint32_t g_clk_after_sdram = 0U;
volatile uint32_t g_clk_after_usb = 0U;
volatile uint32_t g_systick_load_after_config = 0U;
volatile uint32_t g_systick_ctrl_after_config = 0U;

volatile uint32_t g_app_ov5640_init_ret = 0xFFFFFFFFU;
volatile uint32_t g_app_loop_count = 0U;

volatile uint32_t g_usb_configured_seen = 0U;
volatile uint32_t g_usb_last_status = 0U;

volatile uint32_t g_usb_tx_active_dbg = 0U;
volatile uint32_t g_usb_frame_id_dbg = 0U;
volatile uint32_t g_usb_chunk_id_dbg = 0U;
volatile uint32_t g_usb_chunk_total_dbg = 0U;
volatile uint32_t g_usb_payload_len_dbg = 0U;
volatile uint32_t g_usb_send_len_dbg = 0U;
volatile uint32_t g_usb_payload_max_dbg = 0U;
volatile uint32_t g_usb_tx_buf_idx_dbg = 0U;

volatile uint32_t g_usb_send_try_count = 0U;
volatile uint32_t g_usb_send_ok_count = 0U;
volatile uint32_t g_usb_send_busy_count = 0U;
volatile uint32_t g_usb_send_abort_count = 0U;
volatile uint32_t g_usb_send_frame_done_count = 0U;
volatile uint32_t g_usb_drop_frame_count = 0U;
volatile uint32_t g_usb_same_chunk_busy_count_dbg = 0U;
volatile uint32_t g_usb_last_send_ret = 0U;

volatile uint32_t g_frame_service_ok_count = 0U;
volatile uint32_t g_fake_image_fill_done = 0U;

volatile uint32_t g_ai_run_count = 0U;
volatile uint32_t g_ai_obj_count_dbg = 0U;

volatile uint32_t g_ai_best_center_x_dbg = 0U;
volatile uint32_t g_ai_best_center_y_dbg = 0U;
volatile uint32_t g_ai_best_conf_q10000_dbg = 0U;

volatile uint32_t g_ai_track_locked_dbg = 0U;
volatile uint32_t g_ai_track_valid_dbg = 0U;
volatile uint32_t g_ai_track_idx_dbg = 0U;
volatile uint32_t g_ai_track_lost_count_dbg = 0U;
volatile uint32_t g_ai_track_center_x_dbg = 0U;
volatile uint32_t g_ai_track_center_y_dbg = 0U;

volatile uint32_t g_ov5640_frame_interval_us = 0U;
volatile uint32_t g_ov5640_frame_interval_avg_us = 0U;
volatile uint32_t g_ov5640_frame_interval_min_us = 0U;
volatile uint32_t g_ov5640_frame_interval_max_us = 0U;

volatile uint32_t g_ai_infer_time_us = 0U;
volatile uint32_t g_ai_infer_time_avg_us = 0U;
volatile uint32_t g_ai_infer_time_min_us = 0U;
volatile uint32_t g_ai_infer_time_max_us = 0U;

volatile uint32_t g_timer_1s_test_us = 0U;

volatile uint32_t g_gimbal_pid_update_count = 0U;
volatile uint32_t g_gimbal_target_valid_dbg = 0U;
volatile uint32_t g_gimbal_lost_count_dbg = 0U;

volatile uint32_t g_gimbal_target_center_x_dbg = 0U;
volatile uint32_t g_gimbal_target_center_y_dbg = 0U;

volatile int32_t g_gimbal_error_x_dbg = 0;
volatile int32_t g_gimbal_error_y_dbg = 0;

volatile float g_gimbal_x_angle_dbg = GIMBAL_X_INIT_ANGLE;
volatile float g_gimbal_y_angle_dbg = GIMBAL_Y_INIT_ANGLE;
volatile float g_gimbal_x_output_dbg = 0.0f;
volatile float g_gimbal_y_output_dbg = 0.0f;

static uint32_t g_debug_ov5640_last_ts = 0U;
static uint32_t g_debug_ov5640_interval_sample_count = 0U;
static uint32_t g_debug_timer4_test_last_ts = 0U;

#endif

/* ============================================================
   Local prototypes
   ============================================================ */

static void cache_enable(void);
static void mpu_config(void);
static void app_usb_init(void);
static void camera_dma_nvic_config(void);
static void camera_capture_stop(void);
static void app_dcache_clean_by_addr(uint32_t addr, uint32_t size);

#if APP_DEBUG_ENABLE
static void debug_clock_capture(volatile uint32_t *dst);
static void app_debug_timer4_config(void);
static uint32_t app_debug_timer_now_us(void);
static void app_debug_loop_tick(void);
static void app_debug_update_stat(uint32_t value,
                                  volatile uint32_t *cur,
                                  volatile uint32_t *avg,
                                  volatile uint32_t *min,
                                  volatile uint32_t *max,
                                  uint32_t count);
static void app_debug_on_frame_service_ok(void);
static void app_debug_on_ai_done(uint32_t infer_time_us);
#endif

static void fill_fake_rgb565_image(uint32_t frame_addr);

static uint8_t usb_camera_tx_is_idle(void);
static void usb_camera_tx_start(uint32_t frame_addr);
static void usb_camera_tx_step(void);
static uint16_t usb_camera_get_payload_max(uint16_t header_size);
static void usb_drop_current_frame_and_restart(void);

static uint8_t usb_det_tx_step(void);
static void usb_det_build_packet(uint8_t *packet, uint32_t *send_len);
static uint16_t usb_det_conf_to_u16(float conf);
static void usb_det_copy_name(char dst[16], const char *src);

static uint8_t app_ai_object_conf_is_valid(uint32_t obj_idx);
static uint8_t app_ai_select_highest_conf(uint32_t *best_idx);
static uint8_t app_ai_get_object_center_mapped(uint32_t obj_idx, float *cx, float *cy);
static void app_ai_tracker_update(void);
static uint8_t app_ai_get_best_object(uint32_t *best_idx);
static uint8_t app_ai_get_selected_center(uint16_t *cx, uint16_t *cy);
static uint16_t app_ai_map_x(float x);
static uint16_t app_ai_map_y(float y);

static void gimbal_init(void);
static void gimbal_on_ai_result(void);

static void usb_frame_fill_rect(uint32_t frame_addr,
                                int32_t x0,
                                int32_t y0,
                                int32_t x1,
                                int32_t y1,
                                uint16_t color);

static void usb_frame_draw_cross(uint32_t frame_addr,
                                 int32_t cx,
                                 int32_t cy,
                                 int32_t half_len,
                                 int32_t thickness,
                                 uint16_t color);

static void usb_frame_draw_rectangle(uint32_t frame_addr,
                                     int32_t x0,
                                     int32_t y0,
                                     int32_t x1,
                                     int32_t y1,
                                     int32_t thickness,
                                     uint16_t color);

static void usb_draw_center_markers_on_frame(uint32_t frame_addr);

/* ============================================================
   USB packet protocol
   ============================================================ */

#define USB_PKT_TYPE_IMAGE                       0x01U
#define USB_PKT_TYPE_DET                         0x02U

#define USB_IMG_FMT_RGB565                       0x01U
#define USB_DET_FMT_V1                           0x01U
#define USB_DET_FMT_V2                           0x02U

#define USB_CAMERA_FRAME_ADDR                    CAMERA_SHOW_FRAME_ADDR
#define USB_CAMERA_WIDTH                         CAMERA_SHOW_WIDTH
#define USB_CAMERA_HEIGHT                        CAMERA_SHOW_HEIGHT

typedef struct {
    uint16_t magic;
    uint8_t  type;
    uint8_t  format;

    uint32_t frame_id;

    uint16_t width;
    uint16_t height;

    uint16_t chunk_id;
    uint16_t chunk_total;
    uint16_t payload_len;

    uint16_t reserved;
} usb_packet_header_t;

/*
    Detection result object V2.

    Size:
        28 bytes.

    x_angle_delta_deg:
        Unit: degree.

        x_angle_delta_deg = current X servo angle - GIMBAL_X_INIT_ANGLE

        negative -> X servo is left of center
        0        -> X servo is at center
        positive -> X servo is right of center
*/
typedef struct {
    uint16_t cls_index;
    uint16_t object_id;
    uint16_t conf_q10000;
    uint16_t reserved;

    char name[16];

    float x_angle_delta_deg;
} usb_det_object_t;

__ALIGN_BEGIN static uint8_t usb_tx_packet[USB_TX_BUFFER_COUNT][USB_CDC_DATA_PACKET_SIZE] __ALIGN_END;
__ALIGN_BEGIN static uint8_t usb_det_packet[USB_CDC_DATA_PACKET_SIZE] __ALIGN_END;

static uint8_t  g_usb_camera_tx_active = 0U;
static uint8_t  g_usb_det_tx_pending = 0U;

static uint32_t g_usb_camera_frame_id = 0U;
static uint16_t g_usb_camera_chunk_id = 0U;
static uint32_t g_usb_camera_src_addr = USB_CAMERA_FRAME_ADDR;
static uint8_t  g_usb_tx_buf_idx = 0U;

static uint32_t g_usb_same_chunk_busy_count = 0U;
static uint32_t g_ai_frame_divider = 0U;

/* ============================================================
   Lightweight AI tracking state
   ============================================================ */

static uint8_t  g_ai_track_locked = 0U;
static uint8_t  g_ai_track_valid = 0U;
static uint32_t g_ai_track_idx = 0U;
static uint32_t g_ai_track_lost_count = 0U;

static float g_ai_track_center_x = 0.0f;
static float g_ai_track_center_y = 0.0f;

/* ============================================================
   Gimbal PID state
   ============================================================ */

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float last_error;
} app_gimbal_pid_t;

static mg90s_t g_servo_x_pb3;
static mg90s_t g_servo_y_pa15;

static float g_gimbal_x_angle = GIMBAL_X_INIT_ANGLE;
static float g_gimbal_y_angle = GIMBAL_Y_INIT_ANGLE;

static uint32_t g_gimbal_lost_count = 0U;

static app_gimbal_pid_t g_gimbal_x_pid = {
    GIMBAL_X_KP,
    GIMBAL_X_KI,
    GIMBAL_X_KD,
    0.0f,
    0.0f
};

static app_gimbal_pid_t g_gimbal_y_pid = {
    GIMBAL_Y_KP,
    GIMBAL_Y_KI,
    GIMBAL_Y_KD,
    0.0f,
    0.0f
};

/* ============================================================
   Debug helper
   ============================================================ */

#if APP_DEBUG_ENABLE

static void debug_clock_capture(volatile uint32_t *dst)
{
    SystemCoreClockUpdate();
    *dst = SystemCoreClock;
}

static void app_debug_timer4_config(void)
{
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER4);
    timer_deinit(TIMER4);
    timer_struct_para_init(&timer_initpara);

    timer_initpara.prescaler = 299U;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = 0xFFFFFFFFU;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;

    timer_init(TIMER4, &timer_initpara);
    timer_enable(TIMER4);
}

static uint32_t app_debug_timer_now_us(void)
{
    return timer_counter_read(TIMER4);
}

static void app_debug_loop_tick(void)
{
    uint32_t now;
    uint32_t diff;

    g_app_loop_count++;

    now = app_debug_timer_now_us();

    if(0U == g_debug_timer4_test_last_ts) {
        g_debug_timer4_test_last_ts = now;
        return;
    }

    diff = now - g_debug_timer4_test_last_ts;

    if(diff >= 1000000U) {
        g_timer_1s_test_us = diff;
        g_debug_timer4_test_last_ts = now;
    }
}

static void app_debug_update_stat(uint32_t value,
                                  volatile uint32_t *cur,
                                  volatile uint32_t *avg,
                                  volatile uint32_t *min,
                                  volatile uint32_t *max,
                                  uint32_t count)
{
    uint64_t sum;

    *cur = value;

    if(1U == count) {
        *avg = value;
        *min = value;
        *max = value;
        return;
    }

    if(value < *min) {
        *min = value;
    }

    if(value > *max) {
        *max = value;
    }

    sum = ((uint64_t)(*avg) * (uint64_t)(count - 1U)) + (uint64_t)value;
    *avg = (uint32_t)(sum / (uint64_t)count);
}

static void app_debug_on_frame_service_ok(void)
{
    uint32_t now;
    uint32_t interval;

    g_frame_service_ok_count++;

    now = app_debug_timer_now_us();

    if(0U != g_debug_ov5640_last_ts) {
        interval = now - g_debug_ov5640_last_ts;

        g_debug_ov5640_interval_sample_count++;

        app_debug_update_stat(interval,
                              &g_ov5640_frame_interval_us,
                              &g_ov5640_frame_interval_avg_us,
                              &g_ov5640_frame_interval_min_us,
                              &g_ov5640_frame_interval_max_us,
                              g_debug_ov5640_interval_sample_count);
    }

    g_debug_ov5640_last_ts = now;
}

static void app_debug_on_ai_done(uint32_t infer_time_us)
{
    g_ai_run_count++;

    app_debug_update_stat(infer_time_us,
                          &g_ai_infer_time_us,
                          &g_ai_infer_time_avg_us,
                          &g_ai_infer_time_min_us,
                          &g_ai_infer_time_max_us,
                          g_ai_run_count);
}

#endif

/* ============================================================
   Helper
   ============================================================ */

static void app_dcache_clean_by_addr(uint32_t addr, uint32_t size)
{
    uint32_t aligned_addr;
    uint32_t aligned_size;

    if(0U == size) {
        return;
    }

    aligned_addr = addr & ~(uint32_t)(APP_DCACHE_LINE_SIZE - 1U);
    aligned_size = size + (addr - aligned_addr);
    aligned_size = (aligned_size + (APP_DCACHE_LINE_SIZE - 1U)) &
                   ~(uint32_t)(APP_DCACHE_LINE_SIZE - 1U);

    __DSB();
    SCB_CleanDCache_by_Addr((uint32_t *)aligned_addr, (int32_t)aligned_size);
    __DSB();
    __ISB();
}

static void camera_capture_stop(void)
{
    dci_capture_disable();
    dma_channel_disable(DMA1, DMA_CH7);
    dma_interrupt_disable(DMA1, DMA_CH7, DMA_INT_FTF);
    dma_interrupt_flag_clear(DMA1, DMA_CH7, DMA_INT_FLAG_FTF);
}

/* ============================================================
   AI helper + lightweight tracking
   ============================================================ */

static uint8_t app_ai_object_conf_is_valid(uint32_t obj_idx)
{
#if (APP_ENABLE_AI_USB_RESULT && defined(OD_MODEL))
    float conf;

    if(obj_idx >= ai_get_obj_num()) {
        return 0U;
    }

    conf = ai_get_obj_conf(obj_idx);

    if(conf >= AI_TRACK_CONF_MIN) {
        return 1U;
    }

    return 0U;
#else
    (void)obj_idx;
    return 0U;
#endif
}

static uint8_t app_ai_select_highest_conf(uint32_t *best_idx)
{
#if (APP_ENABLE_AI_USB_RESULT && defined(OD_MODEL))
    uint32_t i;
    uint32_t obj_num;
    uint32_t found;
    float conf;
    float best_conf;
    uint32_t best;

    if(best_idx == 0) {
        return 0U;
    }

    obj_num = ai_get_obj_num();

    if(0U == obj_num) {
        return 0U;
    }

    found = 0U;
    best = 0U;
    best_conf = AI_TRACK_CONF_MIN;

    for(i = 0U; i < obj_num; i++) {
        conf = ai_get_obj_conf(i);

        if(conf < AI_TRACK_CONF_MIN) {
            continue;
        }

        if((0U == found) || (conf > best_conf)) {
            found = 1U;
            best_conf = conf;
            best = i;
        }
    }

    if(0U == found) {
        return 0U;
    }

    *best_idx = best;
    return 1U;
#else
    (void)best_idx;
    return 0U;
#endif
}

static uint8_t app_ai_get_object_center_mapped(uint32_t obj_idx, float *cx, float *cy)
{
#if (APP_ENABLE_AI_USB_RESULT && defined(OD_MODEL))
    float x0f;
    float y0f;
    float x1f;
    float y1f;
    float cxf;
    float cyf;

    if((cx == 0) || (cy == 0)) {
        return 0U;
    }

    if(obj_idx >= ai_get_obj_num()) {
        return 0U;
    }

    ai_get_obj_xyxy(obj_idx, &x0f, &y0f, &x1f, &y1f);

    cxf = (x0f + x1f) * 0.5f;
    cyf = (y0f + y1f) * 0.5f;

    *cx = (float)app_ai_map_x(cxf);
    *cy = (float)app_ai_map_y(cyf);

    return 1U;
#else
    (void)obj_idx;
    (void)cx;
    (void)cy;
    return 0U;
#endif
}

static void app_ai_tracker_set_target(uint32_t obj_idx, float cx, float cy)
{
#if (APP_ENABLE_AI_USB_RESULT && AI_TRACK_ENABLE)
    g_ai_track_locked = 1U;
    g_ai_track_valid = 1U;
    g_ai_track_idx = obj_idx;
    g_ai_track_lost_count = 0U;

    g_ai_track_center_x = cx;
    g_ai_track_center_y = cy;

    APP_DEBUG_CODE(
        g_ai_track_locked_dbg = g_ai_track_locked;
        g_ai_track_valid_dbg = g_ai_track_valid;
        g_ai_track_idx_dbg = g_ai_track_idx;
        g_ai_track_lost_count_dbg = g_ai_track_lost_count;
        g_ai_track_center_x_dbg = (uint32_t)(g_ai_track_center_x + 0.5f);
        g_ai_track_center_y_dbg = (uint32_t)(g_ai_track_center_y + 0.5f);
    );
#else
    (void)obj_idx;
    (void)cx;
    (void)cy;
#endif
}

static void app_ai_tracker_mark_lost(void)
{
#if (APP_ENABLE_AI_USB_RESULT && AI_TRACK_ENABLE)
    g_ai_track_valid = 0U;

    if(g_ai_track_lost_count < AI_TRACK_LOST_COUNT_MAX) {
        g_ai_track_lost_count++;
    }

    APP_DEBUG_CODE(
        g_ai_track_locked_dbg = g_ai_track_locked;
        g_ai_track_valid_dbg = g_ai_track_valid;
        g_ai_track_idx_dbg = g_ai_track_idx;
        g_ai_track_lost_count_dbg = g_ai_track_lost_count;
        g_ai_track_center_x_dbg = (uint32_t)(g_ai_track_center_x + 0.5f);
        g_ai_track_center_y_dbg = (uint32_t)(g_ai_track_center_y + 0.5f);
    );
#endif
}

static void app_ai_tracker_update(void)
{
#if (APP_ENABLE_AI_USB_RESULT && AI_TRACK_ENABLE && defined(OD_MODEL))
    uint32_t obj_num;
    uint32_t best_idx;

    float cx;
    float cy;
    float dx;
    float dy;
    float dist_sq;
    float match_dist_sq;

    obj_num = ai_get_obj_num();

    g_ai_track_valid = 0U;

    if(0U == obj_num) {
        app_ai_tracker_mark_lost();
        return;
    }

    if(0U == app_ai_select_highest_conf(&best_idx)) {
        app_ai_tracker_mark_lost();
        return;
    }

    if(0U == app_ai_get_object_center_mapped(best_idx, &cx, &cy)) {
        app_ai_tracker_mark_lost();
        return;
    }

    if(0U == g_ai_track_locked) {
        app_ai_tracker_set_target(best_idx, cx, cy);
        return;
    }

    dx = cx - g_ai_track_center_x;
    dy = cy - g_ai_track_center_y;
    dist_sq = (dx * dx) + (dy * dy);
    match_dist_sq = AI_TRACK_MATCH_MAX_DIST_PX * AI_TRACK_MATCH_MAX_DIST_PX;

    if(dist_sq <= match_dist_sq) {
        g_ai_track_center_x = ((1.0f - AI_TRACK_SMOOTH_ALPHA) * g_ai_track_center_x) +
                              (AI_TRACK_SMOOTH_ALPHA * cx);

        g_ai_track_center_y = ((1.0f - AI_TRACK_SMOOTH_ALPHA) * g_ai_track_center_y) +
                              (AI_TRACK_SMOOTH_ALPHA * cy);

        g_ai_track_idx = best_idx;
        g_ai_track_valid = 1U;
        g_ai_track_lost_count = 0U;

        APP_DEBUG_CODE(
            g_ai_track_locked_dbg = g_ai_track_locked;
            g_ai_track_valid_dbg = g_ai_track_valid;
            g_ai_track_idx_dbg = g_ai_track_idx;
            g_ai_track_lost_count_dbg = g_ai_track_lost_count;
            g_ai_track_center_x_dbg = (uint32_t)(g_ai_track_center_x + 0.5f);
            g_ai_track_center_y_dbg = (uint32_t)(g_ai_track_center_y + 0.5f);
        );

        return;
    }

    app_ai_tracker_set_target(best_idx, cx, cy);
#endif
}

static uint8_t app_ai_get_best_object(uint32_t *best_idx)
{
#if (APP_ENABLE_AI_USB_RESULT && defined(OD_MODEL))
    if(best_idx == 0) {
        return 0U;
    }

#if AI_TRACK_ENABLE
    if((0U != g_ai_track_valid) &&
       (g_ai_track_idx < ai_get_obj_num()) &&
       (0U != app_ai_object_conf_is_valid(g_ai_track_idx))) {
        *best_idx = g_ai_track_idx;
        return 1U;
    }

    return 0U;
#else
    return app_ai_select_highest_conf(best_idx);
#endif

#else
    (void)best_idx;
    return 0U;
#endif
}

static uint8_t app_ai_get_selected_center(uint16_t *cx, uint16_t *cy)
{
#if (APP_ENABLE_AI_USB_RESULT && defined(OD_MODEL))
    uint32_t best_idx;
    float x0f;
    float y0f;
    float x1f;
    float y1f;
    float cxf;
    float cyf;
    float center_x;
    float center_y;

    if((cx == 0) || (cy == 0)) {
        return 0U;
    }

#if AI_TRACK_ENABLE
    if(0U == app_ai_get_best_object(&best_idx)) {
        return 0U;
    }

    center_x = g_ai_track_center_x;
    center_y = g_ai_track_center_y;

    if(center_x < 0.0f) {
        center_x = 0.0f;
    }

    if(center_x > (float)(USB_CAMERA_WIDTH - 1U)) {
        center_x = (float)(USB_CAMERA_WIDTH - 1U);
    }

    if(center_y < 0.0f) {
        center_y = 0.0f;
    }

    if(center_y > (float)(USB_CAMERA_HEIGHT - 1U)) {
        center_y = (float)(USB_CAMERA_HEIGHT - 1U);
    }

    *cx = (uint16_t)(center_x + 0.5f);
    *cy = (uint16_t)(center_y + 0.5f);

    return 1U;
#else
    if(0U == app_ai_get_best_object(&best_idx)) {
        return 0U;
    }

    ai_get_obj_xyxy(best_idx, &x0f, &y0f, &x1f, &y1f);

    cxf = (x0f + x1f) * 0.5f;
    cyf = (y0f + y1f) * 0.5f;

    *cx = app_ai_map_x(cxf);
    *cy = app_ai_map_y(cyf);

    return 1U;
#endif

#else
    (void)cx;
    (void)cy;
    return 0U;
#endif
}

static uint16_t app_ai_map_x(float x)
{
    float mapped;

#if defined(INPUT_WIDTH) && (INPUT_WIDTH > 1)
    mapped = (x * (float)(USB_CAMERA_WIDTH - 1U)) / (float)(INPUT_WIDTH - 1U);
#else
    mapped = x;
#endif

    if(mapped < 0.0f) {
        mapped = 0.0f;
    }

    if(mapped > (float)(USB_CAMERA_WIDTH - 1U)) {
        mapped = (float)(USB_CAMERA_WIDTH - 1U);
    }

    return (uint16_t)(mapped + 0.5f);
}

static uint16_t app_ai_map_y(float y)
{
    float mapped;

#if defined(INPUT_HEIGHT) && (INPUT_HEIGHT > 1)
    mapped = (y * (float)(USB_CAMERA_HEIGHT - 1U)) / (float)(INPUT_HEIGHT - 1U);
#else
    mapped = y;
#endif

    if(mapped < 0.0f) {
        mapped = 0.0f;
    }

    if(mapped > (float)(USB_CAMERA_HEIGHT - 1U)) {
        mapped = (float)(USB_CAMERA_HEIGHT - 1U);
    }

    return (uint16_t)(mapped + 0.5f);
}

/* ============================================================
   Gimbal PID
   ============================================================ */

static float gimbal_abs_float(float x)
{
    if(x < 0.0f) {
        return -x;
    }

    return x;
}

static float gimbal_limit_float(float value, float min_value, float max_value)
{
    if(value < min_value) {
        return min_value;
    }

    if(value > max_value) {
        return max_value;
    }

    return value;
}

static void gimbal_apply_angle(void)
{
#if APP_ENABLE_GIMBAL_PID
    g_gimbal_x_angle = gimbal_limit_float(g_gimbal_x_angle,
                                          GIMBAL_X_ANGLE_MIN,
                                          GIMBAL_X_ANGLE_MAX);

    g_gimbal_y_angle = gimbal_limit_float(g_gimbal_y_angle,
                                          GIMBAL_Y_ANGLE_MIN,
                                          GIMBAL_Y_ANGLE_MAX);

    /*
        Use float angle output.

        PB3:
            mg90s driver automatically uses 270-degree mapping.

        PA15:
            mg90s driver automatically uses 180-degree mapping.
    */
    mg90s_set_angle_float(&g_servo_x_pb3, g_gimbal_x_angle);
    mg90s_set_angle_float(&g_servo_y_pa15, g_gimbal_y_angle);

    APP_DEBUG_CODE(
        g_gimbal_x_angle_dbg = g_gimbal_x_angle;
        g_gimbal_y_angle_dbg = g_gimbal_y_angle;
    );
#endif
}

static void gimbal_pid_reset_integral(void)
{
    g_gimbal_x_pid.integral = 0.0f;
    g_gimbal_x_pid.last_error = 0.0f;

    g_gimbal_y_pid.integral = 0.0f;
    g_gimbal_y_pid.last_error = 0.0f;
}

static float gimbal_pid_calc(app_gimbal_pid_t *pid, float error)
{
    float derivative;
    float output;

    if(pid == 0) {
        return 0.0f;
    }

    if(gimbal_abs_float(error) <= GIMBAL_DEADZONE_PX) {
        pid->integral *= 0.90f;
        pid->last_error = error;
        return 0.0f;
    }

    pid->integral += error;

    if(pid->integral > GIMBAL_INTEGRAL_LIMIT) {
        pid->integral = GIMBAL_INTEGRAL_LIMIT;
    }

    if(pid->integral < -GIMBAL_INTEGRAL_LIMIT) {
        pid->integral = -GIMBAL_INTEGRAL_LIMIT;
    }

    derivative = error - pid->last_error;
    pid->last_error = error;

    output = (pid->kp * error) +
             (pid->ki * pid->integral) +
             (pid->kd * derivative);

    if(output > GIMBAL_MAX_STEP_DEG) {
        output = GIMBAL_MAX_STEP_DEG;
    }

    if(output < -GIMBAL_MAX_STEP_DEG) {
        output = -GIMBAL_MAX_STEP_DEG;
    }

    if((output > 0.0f) && (output < GIMBAL_MIN_STEP_DEG)) {
        output = GIMBAL_MIN_STEP_DEG;
    } else if((output < 0.0f) && (output > -GIMBAL_MIN_STEP_DEG)) {
        output = -GIMBAL_MIN_STEP_DEG;
    } else {
        /* no operation */
    }

    return output;
}

static float gimbal_move_towards(float current, float target, float step)
{
    float diff;

    diff = target - current;

    if(gimbal_abs_float(diff) <= step) {
        return target;
    }

    if(diff > 0.0f) {
        return current + step;
    }

    return current - step;
}

static void gimbal_return_to_init_step(void)
{
    g_gimbal_x_angle = gimbal_move_towards(g_gimbal_x_angle,
                                           GIMBAL_X_INIT_ANGLE,
                                           GIMBAL_RETURN_STEP_DEG);

    g_gimbal_y_angle = gimbal_move_towards(g_gimbal_y_angle,
                                           GIMBAL_Y_INIT_ANGLE,
                                           GIMBAL_RETURN_STEP_DEG);

    gimbal_apply_angle();
}

static void gimbal_init(void)
{
#if APP_ENABLE_GIMBAL_PID
    mg90s_init(&g_servo_x_pb3, GPIOB, GPIO_PIN_3, RCU_GPIOB);
    mg90s_init(&g_servo_y_pa15, GPIOA, GPIO_PIN_15, RCU_GPIOA);

    g_gimbal_x_angle = GIMBAL_X_INIT_ANGLE;
    g_gimbal_y_angle = GIMBAL_Y_INIT_ANGLE;
    g_gimbal_lost_count = 0U;

    gimbal_pid_reset_integral();
    gimbal_apply_angle();

    APP_DEBUG_CODE(
        g_gimbal_pid_update_count = 0U;
        g_gimbal_target_valid_dbg = 0U;
        g_gimbal_lost_count_dbg = 0U;
        g_gimbal_target_center_x_dbg = 0U;
        g_gimbal_target_center_y_dbg = 0U;
        g_gimbal_error_x_dbg = 0;
        g_gimbal_error_y_dbg = 0;
        g_gimbal_x_output_dbg = 0.0f;
        g_gimbal_y_output_dbg = 0.0f;
    );
#endif
}

static void gimbal_on_target_lost(void)
{
#if APP_ENABLE_GIMBAL_PID
    if(g_gimbal_lost_count < GIMBAL_LOST_COUNT_MAX) {
        g_gimbal_lost_count++;
    }

    gimbal_pid_reset_integral();

    APP_DEBUG_CODE(
        g_gimbal_target_valid_dbg = 0U;
        g_gimbal_lost_count_dbg = g_gimbal_lost_count;
        g_gimbal_target_center_x_dbg = 0U;
        g_gimbal_target_center_y_dbg = 0U;
        g_gimbal_error_x_dbg = 0;
        g_gimbal_error_y_dbg = 0;
        g_gimbal_x_output_dbg = 0.0f;
        g_gimbal_y_output_dbg = 0.0f;
    );

    if(g_gimbal_lost_count <= GIMBAL_LOST_HOLD_COUNT) {
        return;
    }

    if(g_gimbal_lost_count >= GIMBAL_LOST_RETURN_COUNT) {
        gimbal_return_to_init_step();
    }
#endif
}

static void gimbal_on_ai_result(void)
{
#if (APP_ENABLE_GIMBAL_PID && APP_ENABLE_AI_USB_RESULT && defined(OD_MODEL))
    uint16_t cx;
    uint16_t cy;
    int32_t image_cx;
    int32_t image_cy;
    int32_t error_x;
    int32_t error_y;
    float x_output;
    float y_output;

    if(0U == app_ai_get_selected_center(&cx, &cy)) {
        gimbal_on_target_lost();
        return;
    }

    image_cx = (int32_t)(USB_CAMERA_WIDTH / 2U);
    image_cy = (int32_t)(USB_CAMERA_HEIGHT / 2U);

    error_x = (int32_t)cx - image_cx;
    error_y = (int32_t)cy - image_cy;

    x_output = gimbal_pid_calc(&g_gimbal_x_pid, (float)error_x) * GIMBAL_X_DIR;
    y_output = gimbal_pid_calc(&g_gimbal_y_pid, (float)error_y) * GIMBAL_Y_DIR;

    g_gimbal_x_angle += x_output;
    g_gimbal_y_angle += y_output;

    g_gimbal_lost_count = 0U;

    gimbal_apply_angle();

    APP_DEBUG_CODE(
        g_gimbal_pid_update_count++;
        g_gimbal_target_valid_dbg = 1U;
        g_gimbal_lost_count_dbg = 0U;
        g_gimbal_target_center_x_dbg = cx;
        g_gimbal_target_center_y_dbg = cy;
        g_gimbal_error_x_dbg = error_x;
        g_gimbal_error_y_dbg = error_y;
        g_gimbal_x_output_dbg = x_output;
        g_gimbal_y_output_dbg = y_output;
    );
#endif
}

/* ============================================================
   MCU-side marker drawing
   ============================================================ */

static void usb_frame_fill_rect(uint32_t frame_addr,
                                int32_t x0,
                                int32_t y0,
                                int32_t x1,
                                int32_t y1,
                                uint16_t color)
{
    volatile uint16_t *frame = (volatile uint16_t *)frame_addr;
    int32_t x;
    int32_t y;
    int32_t tmp;

    if(x0 > x1) {
        tmp = x0;
        x0 = x1;
        x1 = tmp;
    }

    if(y0 > y1) {
        tmp = y0;
        y0 = y1;
        y1 = tmp;
    }

    if((x1 < 0) || (y1 < 0)) {
        return;
    }

    if((x0 >= (int32_t)USB_CAMERA_WIDTH) || (y0 >= (int32_t)USB_CAMERA_HEIGHT)) {
        return;
    }

    if(x0 < 0) {
        x0 = 0;
    }

    if(y0 < 0) {
        y0 = 0;
    }

    if(x1 >= (int32_t)USB_CAMERA_WIDTH) {
        x1 = (int32_t)USB_CAMERA_WIDTH - 1;
    }

    if(y1 >= (int32_t)USB_CAMERA_HEIGHT) {
        y1 = (int32_t)USB_CAMERA_HEIGHT - 1;
    }

    for(y = y0; y <= y1; y++) {
        volatile uint16_t *row;

        row = frame + ((uint32_t)y * USB_CAMERA_WIDTH) + (uint32_t)x0;

        for(x = x0; x <= x1; x++) {
            *row++ = color;
        }
    }
}

static void usb_frame_draw_cross(uint32_t frame_addr,
                                 int32_t cx,
                                 int32_t cy,
                                 int32_t half_len,
                                 int32_t thickness,
                                 uint16_t color)
{
    int32_t half_thick;

    if(thickness <= 0) {
        thickness = 1;
    }

    half_thick = thickness / 2;

    usb_frame_fill_rect(frame_addr,
                        cx - half_len,
                        cy - half_thick,
                        cx + half_len,
                        cy - half_thick + thickness - 1,
                        color);

    usb_frame_fill_rect(frame_addr,
                        cx - half_thick,
                        cy - half_len,
                        cx - half_thick + thickness - 1,
                        cy + half_len,
                        color);
}

static void usb_frame_draw_rectangle(uint32_t frame_addr,
                                     int32_t x0,
                                     int32_t y0,
                                     int32_t x1,
                                     int32_t y1,
                                     int32_t thickness,
                                     uint16_t color)
{
    int32_t tmp;
    int32_t w;
    int32_t h;

    if(thickness <= 0) {
        thickness = 1;
    }

    if(x0 > x1) {
        tmp = x0;
        x0 = x1;
        x1 = tmp;
    }

    if(y0 > y1) {
        tmp = y0;
        y0 = y1;
        y1 = tmp;
    }

    w = x1 - x0 + 1;
    h = y1 - y0 + 1;

    if((w < USB_AI_BOX_MIN_SIZE) || (h < USB_AI_BOX_MIN_SIZE)) {
        return;
    }

    usb_frame_fill_rect(frame_addr, x0, y0, x1, y0 + thickness - 1, color);
    usb_frame_fill_rect(frame_addr, x0, y1 - thickness + 1, x1, y1, color);
    usb_frame_fill_rect(frame_addr, x0, y0, x0 + thickness - 1, y1, color);
    usb_frame_fill_rect(frame_addr, x1 - thickness + 1, y0, x1, y1, color);
}

static void usb_draw_center_markers_on_frame(uint32_t frame_addr)
{
    int32_t image_cx;
    int32_t image_cy;

    image_cx = (int32_t)(USB_CAMERA_WIDTH / 2U);
    image_cy = (int32_t)(USB_CAMERA_HEIGHT / 2U);

    usb_frame_draw_cross(frame_addr,
                         image_cx,
                         image_cy,
                         USB_CENTER_CROSS_HALF_LEN,
                         USB_CENTER_CROSS_THICKNESS,
                         USB_CENTER_CROSS_COLOR);

#if (APP_ENABLE_AI_USB_RESULT && defined(OD_MODEL))
    {
        uint32_t obj_num;
        uint32_t best_idx;

        float x0f;
        float y0f;
        float x1f;
        float y1f;

        uint16_t x0;
        uint16_t y0;
        uint16_t x1;
        uint16_t y1;
        uint16_t cx;
        uint16_t cy;
        uint16_t tmp;

        obj_num = ai_get_obj_num();

        if(0U != app_ai_get_best_object(&best_idx)) {
            ai_get_obj_xyxy(best_idx, &x0f, &y0f, &x1f, &y1f);

            x0 = app_ai_map_x(x0f);
            y0 = app_ai_map_y(y0f);
            x1 = app_ai_map_x(x1f);
            y1 = app_ai_map_y(y1f);

            if(x0 > x1) {
                tmp = x0;
                x0 = x1;
                x1 = tmp;
            }

            if(y0 > y1) {
                tmp = y0;
                y0 = y1;
                y1 = tmp;
            }

            usb_frame_draw_rectangle(frame_addr,
                                     (int32_t)x0,
                                     (int32_t)y0,
                                     (int32_t)x1,
                                     (int32_t)y1,
                                     USB_AI_BOX_THICKNESS,
                                     USB_AI_BOX_COLOR);

#if USB_DRAW_BEST_TARGET_CENTER
            if(0U != app_ai_get_selected_center(&cx, &cy)) {
                usb_frame_fill_rect(frame_addr,
                                    (int32_t)cx - USB_TARGET_RECT_HALF_SIZE,
                                    (int32_t)cy - USB_TARGET_RECT_HALF_SIZE,
                                    (int32_t)cx + USB_TARGET_RECT_HALF_SIZE,
                                    (int32_t)cy + USB_TARGET_RECT_HALF_SIZE,
                                    USB_TARGET_CENTER_COLOR);
            }
#endif

            if(0U != app_ai_get_selected_center(&cx, &cy)) {
                APP_DEBUG_CODE(
                    g_ai_best_center_x_dbg = cx;
                    g_ai_best_center_y_dbg = cy;
                    g_ai_best_conf_q10000_dbg = usb_det_conf_to_u16(ai_get_obj_conf(best_idx));
                    g_ai_obj_count_dbg = obj_num;
                );
            }
        } else {
            APP_DEBUG_CODE(
                g_ai_best_center_x_dbg = 0U;
                g_ai_best_center_y_dbg = 0U;
                g_ai_best_conf_q10000_dbg = 0U;
                g_ai_obj_count_dbg = obj_num;
            );
        }
    }
#else
    APP_DEBUG_CODE(
        g_ai_best_center_x_dbg = 0U;
        g_ai_best_center_y_dbg = 0U;
        g_ai_best_conf_q10000_dbg = 0U;
        g_ai_obj_count_dbg = 0U;
    );
#endif
}

/* ============================================================
   USB image transmit
   ============================================================ */

static uint8_t usb_camera_tx_is_idle(void)
{
    if((0U == g_usb_camera_tx_active) && (0U == g_usb_det_tx_pending)) {
        return 1U;
    }

    return 0U;
}

static void usb_camera_tx_start(uint32_t frame_addr)
{
    g_usb_camera_src_addr = frame_addr;
    g_usb_camera_chunk_id = 0U;

#if APP_ENABLE_AI_USB_RESULT
    g_usb_det_tx_pending = 1U;
#else
    g_usb_det_tx_pending = 0U;
#endif

    g_usb_camera_tx_active = 1U;
    g_usb_same_chunk_busy_count = 0U;
    g_usb_tx_buf_idx = 0U;

    APP_DEBUG_CODE(
        g_usb_tx_active_dbg = 1U;
        g_usb_frame_id_dbg = g_usb_camera_frame_id;
        g_usb_chunk_id_dbg = 0U;
        g_usb_tx_buf_idx_dbg = g_usb_tx_buf_idx;
        g_usb_same_chunk_busy_count_dbg = 0U;
    );
}

static uint16_t usb_camera_get_payload_max(uint16_t header_size)
{
    uint16_t payload_max;

    payload_max = (uint16_t)(USB_CDC_DATA_PACKET_SIZE - header_size);

    if(0U != (payload_max & 1U)) {
        payload_max--;
    }

    if((uint16_t)(payload_max + header_size) >= USB_CDC_DATA_PACKET_SIZE) {
        if(payload_max >= 2U) {
            payload_max = (uint16_t)(payload_max - 2U);
        }
    }

    return payload_max;
}

static void usb_drop_current_frame_and_restart(void)
{
    g_usb_same_chunk_busy_count = 0U;

    g_usb_camera_chunk_id = 0U;
    g_usb_camera_tx_active = 0U;
    g_usb_det_tx_pending = 0U;

    g_usb_camera_frame_id++;

    APP_DEBUG_CODE(
        g_usb_tx_active_dbg = 0U;
        g_usb_frame_id_dbg = g_usb_camera_frame_id;
        g_usb_drop_frame_count++;
        g_usb_same_chunk_busy_count_dbg = 0U;
    );

#if (!APP_TEST_USB_FAKE_IMAGE_ONLY) && (!APP_TEST_OV5640_FREEZE_FRAME_ONLY)
    dci_ov5640_capture_restart();
#endif
}

static void usb_camera_tx_step(void)
{
    usb_packet_header_t *hdr;
    uint8_t *packet;
    uint16_t header_size;
    uint16_t payload_max;
    uint32_t image_total_bytes;
    uint16_t chunk_total;
    uint32_t start_byte;
    uint32_t remain;
    uint16_t payload_len;
    uint8_t *payload;
    uint8_t *src;
    uint32_t send_len;
    uint8_t ret;

    if((0U == g_usb_camera_tx_active) && (0U == g_usb_det_tx_pending)) {
        APP_DEBUG_CODE(g_usb_tx_active_dbg = 0U;);
        return;
    }

    APP_DEBUG_CODE(g_usb_last_status = cdc_acm.dev.cur_status;);

    if(USBD_CONFIGURED != cdc_acm.dev.cur_status) {
        g_usb_camera_tx_active = 0U;
        g_usb_det_tx_pending = 0U;

        APP_DEBUG_CODE(
            g_usb_tx_active_dbg = 0U;
            g_usb_send_abort_count++;
        );

#if (!APP_TEST_USB_FAKE_IMAGE_ONLY) && (!APP_TEST_OV5640_FREEZE_FRAME_ONLY)
        dci_ov5640_capture_restart();
#endif

        return;
    }

    APP_DEBUG_CODE(g_usb_configured_seen = 1U;);

#if APP_ENABLE_AI_USB_RESULT
    if(0U != g_usb_det_tx_pending) {
        if(0U == usb_det_tx_step()) {
            g_usb_same_chunk_busy_count++;

            APP_DEBUG_CODE(g_usb_same_chunk_busy_count_dbg = g_usb_same_chunk_busy_count;);

            if(g_usb_same_chunk_busy_count > USB_BUSY_DROP_THRESHOLD) {
                usb_drop_current_frame_and_restart();
            }

            return;
        }

        g_usb_same_chunk_busy_count = 0U;
        APP_DEBUG_CODE(g_usb_same_chunk_busy_count_dbg = 0U;);
        return;
    }
#endif

    if(0U == g_usb_camera_tx_active) {
        return;
    }

    packet = usb_tx_packet[g_usb_tx_buf_idx];
    hdr = (usb_packet_header_t *)packet;

    header_size = (uint16_t)sizeof(usb_packet_header_t);
    payload_max = usb_camera_get_payload_max(header_size);

    image_total_bytes = USB_CAMERA_WIDTH * USB_CAMERA_HEIGHT * 2U;
    chunk_total = (uint16_t)((image_total_bytes + payload_max - 1U) / payload_max);

    start_byte = (uint32_t)g_usb_camera_chunk_id * payload_max;

    if(start_byte >= image_total_bytes) {
        g_usb_camera_chunk_id = 0U;
        g_usb_camera_tx_active = 0U;
        APP_DEBUG_CODE(g_usb_tx_active_dbg = 0U;);
        return;
    }

    remain = image_total_bytes - start_byte;
    payload_len = (remain > (uint32_t)payload_max) ? payload_max : (uint16_t)remain;

    payload = packet + header_size;
    src = (uint8_t *)(g_usb_camera_src_addr + start_byte);

    hdr->magic = 0xAA55U;
    hdr->type = USB_PKT_TYPE_IMAGE;
    hdr->format = USB_IMG_FMT_RGB565;
    hdr->frame_id = g_usb_camera_frame_id;
    hdr->width = USB_CAMERA_WIDTH;
    hdr->height = USB_CAMERA_HEIGHT;
    hdr->chunk_id = g_usb_camera_chunk_id;
    hdr->chunk_total = chunk_total;
    hdr->payload_len = payload_len;
    hdr->reserved = 0U;

    memcpy(payload, src, payload_len);

    send_len = (uint32_t)header_size + payload_len;

    app_dcache_clean_by_addr((uint32_t)packet, send_len);

    APP_DEBUG_CODE(
        g_usb_send_try_count++;
        g_usb_chunk_id_dbg = g_usb_camera_chunk_id;
        g_usb_chunk_total_dbg = chunk_total;
        g_usb_payload_len_dbg = payload_len;
        g_usb_send_len_dbg = send_len;
        g_usb_payload_max_dbg = payload_max;
        g_usb_tx_buf_idx_dbg = g_usb_tx_buf_idx;
    );

    ret = cdc_acm_user_send(&cdc_acm, packet, send_len);

    APP_DEBUG_CODE(g_usb_last_send_ret = ret;);

    if(0U == ret) {
        g_usb_same_chunk_busy_count = 0U;

        APP_DEBUG_CODE(
            g_usb_send_ok_count++;
            g_usb_same_chunk_busy_count_dbg = 0U;
        );

        g_usb_tx_buf_idx ^= 1U;

        g_usb_camera_chunk_id++;

        if(g_usb_camera_chunk_id >= chunk_total) {
            g_usb_camera_chunk_id = 0U;
            g_usb_camera_frame_id++;
            g_usb_camera_tx_active = 0U;

            APP_DEBUG_CODE(
                g_usb_tx_active_dbg = 0U;
                g_usb_frame_id_dbg = g_usb_camera_frame_id;
                g_usb_send_frame_done_count++;
            );

#if (!APP_TEST_USB_FAKE_IMAGE_ONLY) && (!APP_TEST_OV5640_FREEZE_FRAME_ONLY)
            dci_ov5640_capture_restart();
#endif
        }

#if USB_TX_PACE_DELAY_MS > 0U
        delay_1ms(USB_TX_PACE_DELAY_MS);
#endif
    } else {
        g_usb_same_chunk_busy_count++;

        APP_DEBUG_CODE(
            g_usb_send_busy_count++;
            g_usb_same_chunk_busy_count_dbg = g_usb_same_chunk_busy_count;
        );

        if(g_usb_same_chunk_busy_count > USB_BUSY_DROP_THRESHOLD) {
            usb_drop_current_frame_and_restart();
        }
    }
}

/* ============================================================
   USB detection packet
   ============================================================ */

static uint16_t usb_det_conf_to_u16(float conf)
{
    if(conf < 0.0f) {
        conf = 0.0f;
    }

    if(conf > 1.0f) {
        conf = 1.0f;
    }

    return (uint16_t)(conf * 10000.0f + 0.5f);
}

static void usb_det_copy_name(char dst[16], const char *src)
{
    uint32_t i;

    for(i = 0U; i < 16U; i++) {
        dst[i] = 0;
    }

    if(src == 0) {
        return;
    }

    for(i = 0U; i < 15U; i++) {
        if(src[i] == '\0') {
            break;
        }

        dst[i] = src[i];
    }
}

static void usb_det_build_packet(uint8_t *packet, uint32_t *send_len)
{
    usb_packet_header_t *hdr;
    usb_det_object_t *obj_payload;
    uint32_t best_idx;
    uint32_t obj_count;

    hdr = (usb_packet_header_t *)packet;
    obj_payload = (usb_det_object_t *)(packet + sizeof(usb_packet_header_t));

    obj_count = 0U;

#if (APP_ENABLE_AI_USB_RESULT && defined(OD_MODEL))
    /*
        Only send one selected target.
    */
    if(0U != app_ai_get_best_object(&best_idx)) {
        obj_payload[0].cls_index = (uint16_t)results[best_idx].cls_index;
        obj_payload[0].object_id = 0U;
        obj_payload[0].conf_q10000 = usb_det_conf_to_u16(ai_get_obj_conf(best_idx));
        obj_payload[0].reserved = 0U;
        usb_det_copy_name(obj_payload[0].name, ai_get_obj_name(best_idx));

#if APP_ENABLE_GIMBAL_PID
        /*
            X servo angle delta from center.

            Unit:
                degree

            Meaning:
                x_angle_delta_deg = current_x_angle - GIMBAL_X_INIT_ANGLE

                negative -> X servo is left of center
                0        -> X servo is at center
                positive -> X servo is right of center
        */
        obj_payload[0].x_angle_delta_deg = g_gimbal_x_angle - GIMBAL_X_INIT_ANGLE;
#else
        obj_payload[0].x_angle_delta_deg = 0.0f;
#endif

        obj_count = 1U;
    }
#else
    (void)best_idx;
#endif

    hdr->magic = 0xAA55U;
    hdr->type = USB_PKT_TYPE_DET;
    hdr->format = USB_DET_FMT_V2;
    hdr->frame_id = g_usb_camera_frame_id;
    hdr->width = USB_CAMERA_WIDTH;
    hdr->height = USB_CAMERA_HEIGHT;
    hdr->chunk_id = (uint16_t)obj_count;
    hdr->chunk_total = (uint16_t)obj_count;
    hdr->payload_len = (uint16_t)(obj_count * sizeof(usb_det_object_t));
    hdr->reserved = 0U;

    *send_len = (uint32_t)sizeof(usb_packet_header_t) + hdr->payload_len;

    APP_DEBUG_CODE(g_ai_obj_count_dbg = obj_count;);
}

static uint8_t usb_det_tx_step(void)
{
    uint32_t send_len;
    uint8_t ret;

    if(0U == g_usb_det_tx_pending) {
        return 1U;
    }

    usb_det_build_packet(usb_det_packet, &send_len);

    app_dcache_clean_by_addr((uint32_t)usb_det_packet, send_len);

    APP_DEBUG_CODE(g_usb_send_try_count++;);

    ret = cdc_acm_user_send(&cdc_acm, usb_det_packet, send_len);

    APP_DEBUG_CODE(g_usb_last_send_ret = ret;);

    if(0U == ret) {
        g_usb_det_tx_pending = 0U;

        APP_DEBUG_CODE(
            g_usb_send_ok_count++;
        );

        return 1U;
    }

    APP_DEBUG_CODE(g_usb_send_busy_count++;);

    return 0U;
}

/* ============================================================
   Fake image
   ============================================================ */

static void fill_fake_rgb565_image(uint32_t frame_addr)
{
    uint16_t *p;
    uint32_t x;
    uint32_t y;
    uint16_t color;

    p = (uint16_t *)frame_addr;

    for(y = 0U; y < USB_CAMERA_HEIGHT; y++) {
        for(x = 0U; x < USB_CAMERA_WIDTH; x++) {
            if(x < (USB_CAMERA_WIDTH / 3U)) {
                color = 0xF800U;
            } else if(x < ((USB_CAMERA_WIDTH * 2U) / 3U)) {
                color = 0x07E0U;
            } else {
                color = 0x001FU;
            }

            if((x == y) || ((x + y) == (USB_CAMERA_WIDTH - 1U))) {
                color = 0xFFFFU;
            }

            p[y * USB_CAMERA_WIDTH + x] = color;
        }
    }

#if APP_ENABLE_AI_USB_RESULT
    usb_draw_center_markers_on_frame(frame_addr);
#endif

    APP_DEBUG_CODE(g_fake_image_fill_done = 1U;);
}

/* ============================================================
   Main
   ============================================================ */

int main(void)
{
    mpu_config();
    APP_DEBUG_CODE(g_app_stage = 1U;);

    cache_enable();
    APP_DEBUG_CODE(g_app_stage = 2U;);

    systick_config();

#if APP_DEBUG_ENABLE
    app_debug_timer4_config();
#endif

#if APP_ENABLE_GIMBAL_PID
    gimbal_init();
    APP_DEBUG_CODE(g_app_stage = 12U;);
#endif

    APP_DEBUG_CODE(g_app_stage = 3U;);

    APP_DEBUG_CODE(
        debug_clock_capture(&g_clk_after_systick);
        g_systick_load_after_config = SysTick->LOAD;
        g_systick_ctrl_after_config = SysTick->CTRL;
    );

    exmc_synchronous_dynamic_ram_init(EXMC_SDRAM_DEVICE0);
    delay_1ms(1000U);

    APP_DEBUG_CODE(
        debug_clock_capture(&g_clk_after_sdram);
        g_app_stage = 4U;
    );

    app_usb_init();

    APP_DEBUG_CODE(
        debug_clock_capture(&g_clk_after_usb);
        g_app_stage = 5U;
    );

#if APP_TEST_USB_FAKE_IMAGE_ONLY

    fill_fake_rgb565_image(USB_CAMERA_FRAME_ADDR);
    APP_DEBUG_CODE(g_app_stage = 10U;);

    while(1) {
        APP_DEBUG_CODE(
            app_debug_loop_tick();
            g_app_stage = 9U;
        );

        if(usb_camera_tx_is_idle()) {
            usb_camera_tx_start(USB_CAMERA_FRAME_ADDR);
        }

        usb_camera_tx_step();
    }

#else

    camera_dma_nvic_config();
    APP_DEBUG_CODE(g_app_stage = 6U;);

    APP_DEBUG_CODE(g_app_stage = 7U;);

    APP_DEBUG_CODE(g_app_ov5640_init_ret = dci_ov5640_init(););

#if APP_DEBUG_ENABLE
    if(0x00U != g_app_ov5640_init_ret) {
#else
    if(0x00U != dci_ov5640_init()) {
#endif
        APP_DEBUG_CODE(g_app_stage = 0xFFU;);

        while(1) {
        }
    }

    APP_DEBUG_CODE(g_app_stage = 8U;);

#if APP_ENABLE_AI_USB_RESULT
    AI_Init();
    APP_DEBUG_CODE(g_app_stage = 11U;);
#endif

#if APP_TEST_OV5640_FREEZE_FRAME_ONLY

    while(0U == dci_ov5640_frame_service()) {
        APP_DEBUG_CODE(
            app_debug_loop_tick();
            g_app_stage = 9U;
        );
    }

    APP_DEBUG_CODE(app_debug_on_frame_service_ok(););

    camera_capture_stop();

    while(1) {
        APP_DEBUG_CODE(
            app_debug_loop_tick();
            g_app_stage = 9U;
        );

        if(usb_camera_tx_is_idle()) {
            usb_camera_tx_start(USB_CAMERA_FRAME_ADDR);
        }

        usb_camera_tx_step();
    }

#else

    while(1) {
#if APP_DEBUG_ENABLE
        uint32_t ai_t0;
        uint32_t ai_t1;
#endif

        APP_DEBUG_CODE(
            app_debug_loop_tick();
            g_app_stage = 9U;
        );

        if(usb_camera_tx_is_idle()) {
            if(0U != dci_ov5640_frame_service()) {
                APP_DEBUG_CODE(app_debug_on_frame_service_ok(););

#if APP_ENABLE_AI_USB_RESULT
                if(0U == g_ai_frame_divider) {
#if APP_DEBUG_ENABLE
                    ai_t0 = app_debug_timer_now_us();
#endif

                    AI_Run();

#if APP_DEBUG_ENABLE
                    ai_t1 = app_debug_timer_now_us();
                    app_debug_on_ai_done(ai_t1 - ai_t0);
#endif

#if AI_TRACK_ENABLE
                    app_ai_tracker_update();
#endif

#if APP_ENABLE_GIMBAL_PID
                    gimbal_on_ai_result();
#endif
                }

                g_ai_frame_divider++;

                if(g_ai_frame_divider >= APP_AI_RUN_INTERVAL) {
                    g_ai_frame_divider = 0U;
                }

                usb_draw_center_markers_on_frame(USB_CAMERA_FRAME_ADDR);
#endif

                usb_camera_tx_start(USB_CAMERA_FRAME_ADDR);
            }
        }

        usb_camera_tx_step();
    }

#endif

#endif
}

/* ============================================================
   Cache / MPU
   ============================================================ */

static void cache_enable(void)
{
    SCB_EnableICache();

    SCB_EnableDCache();

    __DSB();
    __ISB();
}

static void mpu_config(void)
{
    mpu_region_init_struct mpu_init_struct;

    mpu_region_struct_para_init(&mpu_init_struct);

    ARM_MPU_Disable();
    ARM_MPU_SetRegion(0U, 0U);

    mpu_init_struct.region_base_address  = 0xC0000000U;
    mpu_init_struct.region_size          = MPU_REGION_SIZE_32MB;
    mpu_init_struct.access_permission    = MPU_AP_FULL_ACCESS;

    mpu_init_struct.access_bufferable    = MPU_ACCESS_BUFFERABLE;
    mpu_init_struct.access_cacheable     = MPU_ACCESS_CACHEABLE;
    mpu_init_struct.access_shareable     = MPU_ACCESS_NON_SHAREABLE;

    mpu_init_struct.region_number        = MPU_REGION_NUMBER1;
    mpu_init_struct.subregion_disable    = 0x00U;
    mpu_init_struct.instruction_exec     = MPU_INSTRUCTION_EXEC_NOT_PERMIT;
    mpu_init_struct.tex_type             = MPU_TEX_TYPE0;

    mpu_region_config(&mpu_init_struct);
    mpu_region_enable();

    ARM_MPU_Enable(MPU_MODE_PRIV_DEFAULT);

    __DSB();
    __ISB();
}

/* ============================================================
   USB init
   ============================================================ */

static void app_usb_init(void)
{
    usb_rcu_config();

    usb_timer_init();

#ifdef USE_USBHS0

#ifdef USE_USB_FS
    usb_para_init(&cdc_acm, USBHS0, USB_SPEED_FULL);
#endif

#ifdef USE_USB_HS
    usb_para_init(&cdc_acm, USBHS0, USB_SPEED_HIGH);
#endif

#endif

#ifdef USE_USBHS1

#ifdef USE_USB_FS
    usb_para_init(&cdc_acm, USBHS1, USB_SPEED_FULL);
#endif

#ifdef USE_USB_HS
    usb_para_init(&cdc_acm, USBHS1, USB_SPEED_HIGH);
#endif

#endif

    usbd_init(&cdc_acm, &cdc_desc, &cdc_class);

#ifdef USE_USB_HS
#ifndef USE_ULPI_PHY

#ifdef USE_USBHS0
    pllusb_rcu_config(USBHS0);
#elif defined USE_USBHS1
    pllusb_rcu_config(USBHS1);
#endif

#endif
#endif

    usb_intr_config();
}

/* ============================================================
   OV5640 DMA NVIC
   ============================================================ */

static void camera_dma_nvic_config(void)
{
    nvic_irq_enable(DMA1_Channel7_IRQn, 2U, 0U);
}