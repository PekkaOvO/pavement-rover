#include "gd32h7xx.h"
#include "drv_usb_hw.h"
#include "cdc_acm_core.h"
#include "systick.h"

#include <stdint.h>
#include <string.h>

/* ============================================================
   USB CDC object
   ============================================================ */

usb_core_driver cdc_acm;

extern uint8_t cdc_acm_user_send(usb_core_driver *pudev,
                                 uint8_t *buf,
                                 uint32_t len);

/* ============================================================
   Protocol definition
   ============================================================ */

#define USB_PKT_MAGIC                         0xAA55U

#define USB_PKT_TYPE_IMAGE                    0x01U
#define USB_PKT_TYPE_DET                      0x02U
#define USB_PKT_TYPE_STATUS                   0x03U

#define USB_IMG_FMT_RGB565                    0x01U
#define USB_STATUS_FMT_TEXT                   0x01U

#define TEST_IMAGE_WIDTH                      256U
#define TEST_IMAGE_HEIGHT                     256U
#define TEST_IMAGE_BYTES                      (TEST_IMAGE_WIDTH * TEST_IMAGE_HEIGHT * 2U)

#define USB_TX_BUFFER_COUNT                   2U

#define USB_STATUS_TEXT                       "GD32_IMX6ULL_COMM_OK"

#define USB_TX_FRAME_INTERVAL_MS              300U

#define APP_DCACHE_LINE_SIZE                  32U

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

/* ============================================================
   Global variables
   ============================================================ */

__ALIGN_BEGIN static uint8_t usb_tx_packet[USB_TX_BUFFER_COUNT][USB_CDC_DATA_PACKET_SIZE] __ALIGN_END;

static uint32_t g_test_frame_id = 0U;
static uint16_t g_test_chunk_id = 0U;
static uint8_t  g_test_tx_buf_idx = 0U;

static uint8_t  g_status_pending = 0U;

/* ============================================================
   Local prototypes
   ============================================================ */

static void app_usb_init(void);
static void app_dcache_clean_by_addr(uint32_t addr, uint32_t size);

static uint16_t usb_get_payload_max(uint16_t header_size);
static uint16_t usb_get_image_chunk_total(void);

static void fake_image_fill_payload(uint8_t *payload,
                                    uint32_t start_byte,
                                    uint16_t payload_len,
                                    uint32_t frame_id);

static uint16_t fake_image_get_pixel(uint32_t x,
                                     uint32_t y,
                                     uint32_t frame_id);

static void usb_protocol_test_step(void);
static uint8_t usb_send_image_chunk_step(void);
static uint8_t usb_send_status_packet_step(void);

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

static uint16_t usb_get_payload_max(uint16_t header_size)
{
    uint16_t payload_max;

    payload_max = (uint16_t)(USB_CDC_DATA_PACKET_SIZE - header_size);

    /*
        RGB565 image payload should keep even length.
    */
    if(0U != (payload_max & 1U)) {
        payload_max--;
    }

    /*
        Avoid send_len exactly equal to USB_CDC_DATA_PACKET_SIZE.
        This follows the previous project behavior and avoids unnecessary ZLP issues.
    */
    if((uint16_t)(payload_max + header_size) >= USB_CDC_DATA_PACKET_SIZE) {
        if(payload_max >= 2U) {
            payload_max = (uint16_t)(payload_max - 2U);
        }
    }

    return payload_max;
}

static uint16_t usb_get_image_chunk_total(void)
{
    uint16_t header_size;
    uint16_t payload_max;

    header_size = (uint16_t)sizeof(usb_packet_header_t);
    payload_max = usb_get_payload_max(header_size);

    return (uint16_t)((TEST_IMAGE_BYTES + payload_max - 1U) / payload_max);
}

/* ============================================================
   Fake 256x256 RGB565 image
   ============================================================ */

static uint16_t fake_image_get_pixel(uint32_t x,
                                     uint32_t y,
                                     uint32_t frame_id)
{
    uint16_t color;

    /*
        256x256 RGB565 test pattern:

        left   1/3: red
        middle 1/3: green
        right  1/3: blue

        diagonal lines: white
        moving vertical line: yellow
    */
    if(x < (TEST_IMAGE_WIDTH / 3U)) {
        color = 0xF800U;
    } else if(x < ((TEST_IMAGE_WIDTH * 2U) / 3U)) {
        color = 0x07E0U;
    } else {
        color = 0x001FU;
    }

    if((x == y) || ((x + y) == (TEST_IMAGE_WIDTH - 1U))) {
        color = 0xFFFFU;
    }

    if(x == (frame_id & 0xFFU)) {
        color = 0xFFE0U;
    }

    return color;
}

static void fake_image_fill_payload(uint8_t *payload,
                                    uint32_t start_byte,
                                    uint16_t payload_len,
                                    uint32_t frame_id)
{
    uint32_t i;
    uint32_t absolute_byte;
    uint32_t pixel_index;
    uint32_t x;
    uint32_t y;
    uint16_t pixel;

    for(i = 0U; i < payload_len; i++) {
        absolute_byte = start_byte + i;
        pixel_index = absolute_byte / 2U;

        x = pixel_index % TEST_IMAGE_WIDTH;
        y = pixel_index / TEST_IMAGE_WIDTH;

        pixel = fake_image_get_pixel(x, y, frame_id);

        /*
            RGB565 little-endian:
                low byte first
                high byte second
        */
        if(0U == (absolute_byte & 1U)) {
            payload[i] = (uint8_t)(pixel & 0xFFU);
        } else {
            payload[i] = (uint8_t)((pixel >> 8U) & 0xFFU);
        }
    }
}

/* ============================================================
   USB protocol test sender
   ============================================================ */

static uint8_t usb_send_image_chunk_step(void)
{
    usb_packet_header_t *hdr;
    uint8_t *packet;
    uint8_t *payload;

    uint16_t header_size;
    uint16_t payload_max;
    uint16_t chunk_total;
    uint32_t start_byte;
    uint32_t remain;
    uint16_t payload_len;
    uint32_t send_len;
    uint8_t ret;

    if(USBD_CONFIGURED != cdc_acm.dev.cur_status) {
        return 0U;
    }

    packet = usb_tx_packet[g_test_tx_buf_idx];
    hdr = (usb_packet_header_t *)packet;

    header_size = (uint16_t)sizeof(usb_packet_header_t);
    payload_max = usb_get_payload_max(header_size);
    chunk_total = usb_get_image_chunk_total();

    start_byte = (uint32_t)g_test_chunk_id * payload_max;

    if(start_byte >= TEST_IMAGE_BYTES) {
        g_test_chunk_id = 0U;
        g_status_pending = 1U;
        return 1U;
    }

    remain = TEST_IMAGE_BYTES - start_byte;

    if(remain > (uint32_t)payload_max) {
        payload_len = payload_max;
    } else {
        payload_len = (uint16_t)remain;
    }

    payload = packet + header_size;

    hdr->magic       = USB_PKT_MAGIC;
    hdr->type        = USB_PKT_TYPE_IMAGE;
    hdr->format      = USB_IMG_FMT_RGB565;
    hdr->frame_id    = g_test_frame_id;
    hdr->width       = TEST_IMAGE_WIDTH;
    hdr->height      = TEST_IMAGE_HEIGHT;
    hdr->chunk_id    = g_test_chunk_id;
    hdr->chunk_total = chunk_total;
    hdr->payload_len = payload_len;
    hdr->reserved    = 0U;

    fake_image_fill_payload(payload,
                            start_byte,
                            payload_len,
                            g_test_frame_id);

    send_len = (uint32_t)header_size + payload_len;

    app_dcache_clean_by_addr((uint32_t)packet, send_len);

    ret = cdc_acm_user_send(&cdc_acm, packet, send_len);

    if(0U == ret) {
        g_test_tx_buf_idx ^= 1U;
        g_test_chunk_id++;

        if(g_test_chunk_id >= chunk_total) {
            g_test_chunk_id = 0U;
            g_status_pending = 1U;
        }

        return 1U;
    }

    return 0U;
}

static uint8_t usb_send_status_packet_step(void)
{
    usb_packet_header_t *hdr;
    uint8_t *packet;
    uint8_t *payload;

    uint16_t header_size;
    uint16_t payload_len;
    uint32_t send_len;
    uint8_t ret;

    if(USBD_CONFIGURED != cdc_acm.dev.cur_status) {
        return 0U;
    }

    packet = usb_tx_packet[g_test_tx_buf_idx];
    hdr = (usb_packet_header_t *)packet;
    payload = packet + sizeof(usb_packet_header_t);

    header_size = (uint16_t)sizeof(usb_packet_header_t);
    payload_len = (uint16_t)(sizeof(USB_STATUS_TEXT) - 1U);

    hdr->magic       = USB_PKT_MAGIC;
    hdr->type        = USB_PKT_TYPE_STATUS;
    hdr->format      = USB_STATUS_FMT_TEXT;
    hdr->frame_id    = g_test_frame_id;
    hdr->width       = TEST_IMAGE_WIDTH;
    hdr->height      = TEST_IMAGE_HEIGHT;
    hdr->chunk_id    = 0U;
    hdr->chunk_total = 1U;
    hdr->payload_len = payload_len;
    hdr->reserved    = 0x0000U;

    memcpy(payload, USB_STATUS_TEXT, payload_len);

    send_len = (uint32_t)header_size + payload_len;

    app_dcache_clean_by_addr((uint32_t)packet, send_len);

    ret = cdc_acm_user_send(&cdc_acm, packet, send_len);

    if(0U == ret) {
        g_test_tx_buf_idx ^= 1U;

        /*
            One full 256x256 image frame and its STATUS flag have been sent.
            Next frame_id starts after this.
        */
        g_status_pending = 0U;
        g_test_frame_id++;

        delay_1ms(USB_TX_FRAME_INTERVAL_MS);

        return 1U;
    }

    return 0U;
}

static void usb_protocol_test_step(void)
{
    if(USBD_CONFIGURED != cdc_acm.dev.cur_status) {
        g_test_chunk_id = 0U;
        g_status_pending = 0U;
        return;
    }

    if(0U != g_status_pending) {
        (void)usb_send_status_packet_step();
    } else {
        (void)usb_send_image_chunk_step();
    }
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
   Main
   ============================================================ */

int main(void)
{
    /*
        For this communication test:
            no camera
            no AI
            no SDRAM
            no servo
    */

    systick_config();

    app_usb_init();

    while(1) {
        usb_protocol_test_step();
    }
}