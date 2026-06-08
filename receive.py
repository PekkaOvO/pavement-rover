# -*- coding: utf-8 -*-
"""
PC测试脚本：目标检测 + 流程标志 + 激光雷达距离显示

适配GD32 USB CDC统一包头：
    header = <HBBHHHHHH, 16 bytes
    magic = 0xAA55

支持包：
    IMAGE:
        type=0x01, event=0x01, payload=RGB565 image chunk

    DET/FLOW:
        type=0x02, event=0x00, 普通目标/无目标
        type=0x02, event=0x01, STOP_CAR
        type=0x02, event=0x02, X_DELTA_READY, payload=usb_det_object_t
        type=0x02, event=0x03, FLOW_END

    LASER DEBUG:
        type=0x02, event=0x04, payload=usb_laser_object_t, 16 bytes
        payload = <IiHHBBBB>
            uint32_t time_ms
            int32_t  distance_mm
            uint16_t status
            uint16_t signal
            uint8_t  precision_cm
            uint8_t  interface_mode
            uint8_t  module_id
            uint8_t  valid

运行：
    python pc_target_laser_flow_test.py --port COM15

常用：
    python pc_target_laser_flow_test.py --port COM15 --no-window
    python pc_target_laser_flow_test.py --port COM15 --print-all-target
    python pc_target_laser_flow_test.py --port COM15 --laser-every 5
"""

import argparse
import struct
import time
from dataclasses import dataclass

import serial

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except Exception:
    HAS_CV2 = False


DEFAULT_PORT = "COM15"
DEFAULT_BAUDRATE = 115200

MAGIC = 0xAA55
MAGIC_BYTES = b"\x55\xAA"

HEADER_FMT = "<HBBHHHHHH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

DET_FMT = "<HHHH16sf"
DET_SIZE = struct.calcsize(DET_FMT)

LASER_FMT = "<IiHHBBBB"
LASER_SIZE = struct.calcsize(LASER_FMT)

PKT_IMAGE = 0x01
PKT_DET = 0x02

EVT_IMAGE_RGB565 = 0x01

EVT_NONE = 0x00
EVT_STOP = 0x01
EVT_X_DELTA = 0x02
EVT_FLOW_END = 0x03
EVT_LASER = 0x04

MAX_PAYLOAD_LEN = 4096
MAX_RX_BUF = 1024 * 1024

WINDOW_NAME = "GD32 target + laser flow test"


@dataclass
class Target:
    cls_index: int = 0
    object_id: int = 0
    conf_q10000: int = 0
    name: str = "--"
    x_delta_deg: float = 0.0

    @property
    def conf(self):
        return self.conf_q10000 / 10000.0


@dataclass
class Laser:
    time_ms: int = 0
    distance_mm: int = 0
    status: int = 0
    signal: int = 0
    precision_cm: int = 0
    mode: int = 0
    module_id: int = 0
    valid: int = 0


class FlowTracker:
    def __init__(self):
        self.image_ok = False
        self.target_ok = False
        self.stop_count = 0
        self.x_delta_ok = False
        self.flow_end_ok = False
        self.laser_ok = False
        self.last_target = Target()
        self.last_laser = Laser()
        self.last_target_print = 0.0
        self.last_laser_print = 0.0
        self.laser_print_count = 0

    def summary(self):
        print("\n========== 流程检查汇总 ==========")
        print(f"图像接收        : {'OK' if self.image_ok else 'NO'}")
        print(f"目标信息        : {'OK' if self.target_ok else 'NO'}")
        print(f"激光距离        : {'OK' if self.laser_ok else 'NO'}")
        print(f"STOP_CAR次数    : {self.stop_count}")
        print(f"X_DELTA_READY   : {'OK' if self.x_delta_ok else 'NO'}")
        print(f"FLOW_END        : {'OK' if self.flow_end_ok else 'NO'}")
        if self.laser_ok:
            print(f"最后距离        : {self.last_laser.distance_mm} mm")
        print("==================================\n")


def parse_packet(buf: bytearray):
    pos = buf.find(MAGIC_BYTES)
    if pos < 0:
        if len(buf) > 1:
            del buf[:-1]
        return None

    if pos > 0:
        del buf[:pos]
        return "resync"

    if len(buf) < HEADER_SIZE:
        return None

    try:
        magic, typ, evt, width, height, pid, total, plen, reserved = struct.unpack_from(HEADER_FMT, buf, 0)
    except struct.error:
        return None

    if magic != MAGIC:
        del buf[0]
        return "resync"

    if plen > MAX_PAYLOAD_LEN:
        del buf[0]
        return "bad"

    need = HEADER_SIZE + plen
    if len(buf) < need:
        return None

    payload = bytes(buf[HEADER_SIZE:need])
    del buf[:need]

    return {
        "type": typ,
        "event": evt,
        "width": width,
        "height": height,
        "packet_id": pid,
        "packet_total": total,
        "payload_len": plen,
        "payload": payload,
    }


def decode_target(payload: bytes):
    if len(payload) < DET_SIZE:
        return None

    cls_index, object_id, conf_q10000, _reserved, name_raw, x_delta = struct.unpack_from(DET_FMT, payload, 0)
    name = name_raw.split(b"\x00", 1)[0].decode("ascii", errors="replace")
    if not name:
        name = f"cls_{cls_index}"

    return Target(cls_index, object_id, conf_q10000, name, x_delta)


def decode_laser(payload: bytes):
    if len(payload) < LASER_SIZE:
        return None

    t_ms, distance_mm, status, signal, precision_cm, mode, module_id, valid = struct.unpack_from(
        LASER_FMT, payload, 0
    )

    return Laser(t_ms, distance_mm, status, signal, precision_cm, mode, module_id, valid)


def rgb565_to_bgr(data: bytes, width: int, height: int):
    if not HAS_CV2:
        return None

    if len(data) != width * height * 2:
        return None

    raw = np.frombuffer(data, dtype="<u2").reshape((height, width))
    r = ((raw >> 11) & 0x1F).astype(np.uint16)
    g = ((raw >> 5) & 0x3F).astype(np.uint16)
    b = (raw & 0x1F).astype(np.uint16)

    r = (r * 255 // 31).astype(np.uint8)
    g = (g * 255 // 63).astype(np.uint8)
    b = (b * 255 // 31).astype(np.uint8)

    return np.dstack((b, g, r))


def draw_overlay(img, target: Target, laser: Laser, flow: FlowTracker):
    if img is None or not HAS_CV2:
        return img

    laser_text = "--"
    if flow.laser_ok:
        laser_text = f"{laser.distance_mm}mm valid={laser.valid} st={laser.status} sig={laser.signal}"

    lines = [
        f"class: {target.name}  conf: {target.conf:.2f}",
        f"laser: {laser_text}",
        f"STOP:{flow.stop_count}  X_DELTA:{'Y' if flow.x_delta_ok else 'N'}  END:{'Y' if flow.flow_end_ok else 'N'}",
    ]

    y = 22
    for text in lines:
        cv2.putText(img, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 0), 1, cv2.LINE_AA)
        y += 20

    return img


def print_event_stop(flow: FlowTracker):
    flow.stop_count += 1

    if flow.stop_count == 1:
        print("\n========== 收到标志：STOP_CAR #1 ==========")
        print("含义：AI目标稳定，GD32要求IMX6ULL停车。")
        print("下一步：等待、云台居中，然后应出现 X_DELTA_READY。")
        print("==========================================\n")
    elif flow.stop_count == 2:
        print("\n========== 收到标志：STOP_CAR #2 ==========")
        print("含义：激光距离差值判断检测到裂缝，GD32要求IMX6ULL再次停车。")
        print("下一步：Y轴俯拍，之后应出现 FLOW_END。")
        print("==========================================\n")
    else:
        print(f"\n========== 收到标志：STOP_CAR #{flow.stop_count} ==========\n")


def print_event_x_delta(target):
    print("\n========== 收到标志：X_DELTA_READY ==========")
    if target is not None:
        print(f"目标类别：{target.name}")
        print(f"置信度  ：{target.conf:.2f}")
        print(f"X角度差 ：{target.x_delta_deg:.2f} deg")
    else:
        print("警告：X_DELTA_READY 未携带有效目标payload。")
    print("含义：云台居中稳定，角度差已发送给IMX6ULL。")
    print("下一步：IMX6ULL调整车身并前进，GD32开始激光裂缝检测。")
    print("============================================\n")


def print_event_flow_end(flow: FlowTracker):
    print("\n========== 收到标志：FLOW_END ==========")
    print("含义：裂缝停车 + Y轴俯拍等待完成，完整流程跑通。")
    print("=======================================\n")
    flow.summary()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--timeout", type=float, default=0.01)
    parser.add_argument("--read-size", type=int, default=4096)
    parser.add_argument("--no-window", action="store_true")
    parser.add_argument("--print-all-target", action="store_true", help="每个普通目标包都打印")
    parser.add_argument("--laser-every", type=int, default=1, help="每N个激光包打印一次距离")
    parser.add_argument("--stats-interval", type=float, default=3.0)
    args = parser.parse_args()

    show_window = (not args.no_window) and HAS_CV2

    print("正在打开 USB CDC 串口...")
    print(f"  串口          : {args.port}")
    print(f"  波特率        : {args.baudrate}")
    print(f"  包头格式      : {HEADER_FMT}, size={HEADER_SIZE}")
    print(f"  目标payload   : {DET_FMT}, size={DET_SIZE}")
    print(f"  激光payload   : {LASER_FMT}, size={LASER_SIZE}")
    print("  目标事件      : STOP_CAR / X_DELTA_READY / FLOW_END")
    print("  激光显示      : event=0x04 时打印 distance/status/signal")
    print("  退出          : Ctrl+C" + (" 或窗口按 q/ESC" if show_window else ""))

    if not HAS_CV2 and not args.no_window:
        print("提示：未安装 opencv-python/numpy，所以不显示窗口。需要窗口请执行：pip install opencv-python numpy")

    ser = serial.Serial(args.port, args.baudrate, timeout=args.timeout, write_timeout=0.001)

    buf = bytearray()
    flow = FlowTracker()

    image_chunks = []
    image_total = 0
    image_w = 0
    image_h = 0

    total_bytes = 0
    stat_bytes = 0
    stat_img = 0
    stat_evt = 0
    stat_laser = 0
    bad = 0
    resync = 0
    frames = 0
    last_stat = time.time()

    try:
        while True:
            waiting = ser.in_waiting
            data = ser.read(waiting if waiting > 0 else args.read_size)

            if data:
                buf.extend(data)
                total_bytes += len(data)
                stat_bytes += len(data)

            if len(buf) > MAX_RX_BUF:
                pos = buf.rfind(MAGIC_BYTES)
                if pos >= 0:
                    del buf[:pos]
                else:
                    buf.clear()

            while True:
                pkt = parse_packet(buf)
                if pkt is None:
                    break
                if pkt == "resync":
                    resync += 1
                    continue
                if pkt == "bad":
                    bad += 1
                    continue

                typ = pkt["type"]
                evt = pkt["event"]

                if typ == PKT_IMAGE and evt == EVT_IMAGE_RGB565:
                    stat_img += 1
                    w, h = pkt["width"], pkt["height"]
                    pid, total = pkt["packet_id"], pkt["packet_total"]

                    if total == 0:
                        bad += 1
                        continue

                    if pid == 0:
                        image_w, image_h, image_total = w, h, total
                        image_chunks = [None] * total

                    if image_chunks and pid < len(image_chunks):
                        image_chunks[pid] = pkt["payload"]

                    if image_chunks and all(x is not None for x in image_chunks):
                        frames += 1
                        frame_bytes = b"".join(image_chunks)
                        image_chunks = []

                        if not flow.image_ok:
                            flow.image_ok = True
                            print(f"图像提示：已成功拼出第一帧，size={image_w}x{image_h}, chunks={image_total}")

                        if show_window:
                            img = rgb565_to_bgr(frame_bytes, image_w, image_h)
                            if img is not None:
                                img = draw_overlay(img, flow.last_target, flow.last_laser, flow)
                                cv2.imshow(WINDOW_NAME, img)
                                key = cv2.waitKey(1) & 0xFF
                                if key in (ord("q"), 27):
                                    raise KeyboardInterrupt

                elif typ == PKT_DET:
                    stat_evt += 1

                    if evt == EVT_NONE:
                        target = decode_target(pkt["payload"])
                        if target is not None:
                            flow.last_target = target
                            now = time.time()

                            need_print = args.print_all_target
                            if not flow.target_ok:
                                flow.target_ok = True
                                need_print = True
                            if now - flow.last_target_print >= 2.0:
                                need_print = True

                            if need_print:
                                print(f"目标信息：class={target.name}, conf={target.conf:.2f}")
                                flow.last_target_print = now

                    elif evt == EVT_STOP:
                        print_event_stop(flow)

                    elif evt == EVT_X_DELTA:
                        target = decode_target(pkt["payload"])
                        if target is not None:
                            flow.last_target = target
                        flow.x_delta_ok = True
                        print_event_x_delta(target)

                    elif evt == EVT_FLOW_END:
                        flow.flow_end_ok = True
                        print_event_flow_end(flow)

                    elif evt == EVT_LASER:
                        laser = decode_laser(pkt["payload"])
                        if laser is not None:
                            flow.last_laser = laser
                            flow.laser_ok = True
                            flow.laser_print_count += 1
                            stat_laser += 1

                            if args.laser_every <= 1 or (flow.laser_print_count % args.laser_every == 0):
                                print(
                                    f"雷达距离：{laser.distance_mm} mm | "
                                    f"valid={laser.valid} | status={laser.status} | "
                                    f"signal={laser.signal} | precision={laser.precision_cm} cm | "
                                    f"id={laser.module_id} | t={laser.time_ms} ms"
                                )
                        else:
                            print("雷达提示：收到event=0x04，但payload长度不足。")

                    else:
                        print(f"提示：未知DET事件 event=0x{evt:02X}, payload_len={pkt['payload_len']}")

            now = time.time()
            if now - last_stat >= args.stats_interval:
                dt = now - last_stat
                rx_rate = int(stat_bytes / dt) if dt > 0 else 0
                print(
                    f"状态统计：接收={rx_rate} B/s，完整帧={frames}，"
                    f"图像包={stat_img / dt:.1f}/s，事件包={stat_evt / dt:.1f}/s，"
                    f"雷达包={stat_laser / dt:.1f}/s，STOP={flow.stop_count}，"
                    f"X_DELTA={'OK' if flow.x_delta_ok else 'NO'}，"
                    f"FLOW_END={'OK' if flow.flow_end_ok else 'NO'}，"
                    f"坏包={bad}，重同步={resync}，缓存={len(buf)}"
                )
                stat_bytes = 0
                stat_img = 0
                stat_evt = 0
                stat_laser = 0
                last_stat = now

    except KeyboardInterrupt:
        pass
    finally:
        print("\n正在退出...")
        flow.summary()
        try:
            ser.close()
        except Exception:
            pass
        if HAS_CV2:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass


if __name__ == "__main__":
    main()
