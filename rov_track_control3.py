#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import time
from collections import deque
import struct
import pygame

# ========== 下位机 USART6 协议（与你给的 Usermain.cpp 一致） ==========
# 帧格式: [HEAD][CODE][LEN][PAYLOAD...]
PI_IP = "192.168.138.2"
PI_PORT = 5000

# 这两个 HEAD 必须与下位机 device[] 下标一致
PROPELLER_FRAME_HEAD = 0x02
SERVO_FRAME_HEAD = 0x03

CMD_SET_DEPTH_VALUE = 0x00  # payload: int16
CMD_SET_DEPTH_INCRE = 0x01  # payload: int16
CMD_SET_DEPTH_FORCE = 0x02  # payload: int16

CMD_TOGGLE_FLOAT = 0x10     # payload: none
CMD_TOGGLE_YAW_LOOP = 0x20  # payload: none

CMD_SET_YAW_VALUE = 0x21
CMD_INC_YAW_INCRE = 0x22
CMD_SET_YAW_RATE = 0x23       # payload: int16 deg/s

CMD_SET_PLANAR_VEL = 0x30     # payload: int8 x, int8 y

MAX_SPEED = 99
DIRECTION_THRESHOLD = 0.1
ROTATE_THRESHOLD = 0.2
DEPTH_THRESHOLD = 0.2

# ========== TCP 连接 ==========
sock = None


def log_send(msg: str) -> None:
    print("[SEND]", msg)


def connect_to_pi() -> None:
    global sock
    while True:
        try:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((PI_IP, PI_PORT))
            sock = s
            print(f"[NET] Connected to {PI_IP}:{PI_PORT}")
            return
        except Exception as e:
            print(f"[NET] Connect failed: {e}, retrying...")
            time.sleep(1)


def send_pkt(head: int, code: int, payload: bytes = b"") -> None:
    """下位机匹配格式: [HEAD][CODE][LEN][PAYLOAD...]"""
    global sock
    if payload is None:
        payload = b""
    ln = len(payload) & 0xFF
    frame = bytes([head & 0xFF, code & 0xFF, ln]) + payload
    try:
        sock.sendall(frame)
        log_send(frame.hex(" "))
    except Exception as e:
        print(f"[NET] Send failed: {e}")
        connect_to_pi()


def i16le(val: int) -> bytes:
    return struct.pack("<h", int(val))


def calculate_speed(axis_value: float, threshold: float) -> int:
    speed = 0
    if not -threshold < axis_value < threshold:
        if axis_value > 0:
            speed = int((axis_value - threshold) * MAX_SPEED / (1 - threshold))
        else:
            speed = -int((-axis_value - threshold) * MAX_SPEED / (1 - threshold))
    return speed


def send_xy(x: int, y: int) -> None:
    x = max(min(int(x), 99), -99)
    y = max(min(int(y), 99), -99)
    payload = bytes([(x & 0xFF), (y & 0xFF)])
    send_pkt(PROPELLER_FRAME_HEAD, CMD_SET_PLANAR_VEL, payload)

def send_xy_2(x: int, y: int) -> None:
    x = max(min(int(x), 99), -99)
    y = max(min(int(y), 99), -99)
    payload = bytes([(y & 0xFF), (x & 0xFF)])
    send_pkt(PROPELLER_FRAME_HEAD, CMD_SET_PLANAR_VEL, payload)

# ========== GStreamer + GI 视频输入 ==========
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst
import numpy as np
import cv2

Gst.init(None)

PIPELINE = (
    "udpsrc port=5600 ! "
    "application/x-rtp,encoding-name=H264,payload=96 ! "
    "rtpjitterbuffer latency=40 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! "
    "video/x-raw,format=BGR ! appsink name=mysink max-buffers=1 drop=true"
)

pipeline = Gst.parse_launch(PIPELINE)
appsink = pipeline.get_by_name("mysink")
appsink.set_property("emit-signals", True)
appsink.set_property("sync", False)

pipeline.set_state(Gst.State.PLAYING)
print("[GStreamer] Video pipeline started")


def gst_read():
    sample = appsink.emit("pull-sample")
    if sample is None:
        return None
    buf = sample.get_buffer()
    caps = sample.get_caps().get_structure(0)
    w = caps.get_value("width")
    h = caps.get_value("height")

    ok, mapinfo = buf.map(Gst.MapFlags.READ)
    if not ok:
        return None

    frame = np.frombuffer(mapinfo.data, np.uint8).reshape((h, w, 3))
    buf.unmap(mapinfo)
    return frame


# ========== CSRT 跟踪 ==========
tracker = cv2.legacy.TrackerCSRT_create()
tracking = False
tracking_paused = False
bbox = None

# ========== 自动控制参数 ==========
TARGET_DIST = 0.5
W_AT_TARGET = 90.0

CENTER_X = 640 / 2
CENTER_Y = 360 / 2

CENTER_DEAD_X = 2
CENTER_DEAD_Y = 2

MAX_X_VEL = 20
MAX_Y_VEL = 40

WIDTH_WINDOW = 6
width_history = deque(maxlen=WIDTH_WINDOW)

CTRL_INTERVAL = 0.05
last_ctrl_time = 0.0

# 自动艏摇（Yaw rate）参数
AUTO_YAW_DEAD_PX = 8
AUTO_YAW_RATE_LIMIT = 30       # deg/s
AUTO_YAW_KP = 0.09             # px -> deg/s 的比例
yaw_rate_filt = 0.0
YAW_ALPHA = 0.5

# 自动上下（Depth force）参数
AUTO_DEPTH_DEAD_PX = 10
AUTO_DEPTH_FORCE_LIMIT = 45    # 与下位机推力标度一致时再调
AUTO_DEPTH_KP = 0.14           # px -> force 的比例
depth_force_filt = 0.0
DEPTH_ALPHA = 0.45


def calc_planar_control(cx: float, w_smooth: float):
    """自动平面：根据水平误差和距离误差输出 x/y"""
    dx = cx - CENTER_X
    if abs(dx) < CENTER_DEAD_X:
        y_vel = 0
    else:
        y_vel = dx * 0.24
        y_vel = max(min(int(y_vel), MAX_Y_VEL), -MAX_Y_VEL)

    w_smooth = max(float(w_smooth), 1.0)  # 防止除零
    dist = TARGET_DIST * (W_AT_TARGET / w_smooth)
    diff = TARGET_DIST - dist
    if abs(diff) < 0.08:
        x_vel = 0
    else:
        x_vel = -diff * 55.0
        x_vel = max(min(int(x_vel), MAX_X_VEL), -MAX_X_VEL)

    return x_vel, y_vel, dist


def calc_auto_yaw_rate(cx: float) -> int:
    """自动艏摇：根据目标在图像水平位置控制 yaw rate"""
    dx = cx - CENTER_X
    if abs(dx) < AUTO_YAW_DEAD_PX:
        cmd = 0
    else:
        cmd = int(-AUTO_YAW_KP * dx)  # dx>0 目标在右侧，负号表示向右转（依你下位机定义可再反）
        cmd = max(min(cmd, AUTO_YAW_RATE_LIMIT), -AUTO_YAW_RATE_LIMIT)

    global yaw_rate_filt
    yaw_rate_filt = YAW_ALPHA * cmd + (1.0 - YAW_ALPHA) * yaw_rate_filt
    return int(yaw_rate_filt)


def calc_auto_depth_force(cy: float) -> int:
    """自动上下：根据目标在图像垂直位置控制 depth force（推力）"""
    dy = cy - CENTER_Y
    if abs(dy) < AUTO_DEPTH_DEAD_PX:
        cmd = 0
    else:
        # dy>0 目标在画面下方，通常需要下潜或上浮取决于相机安装方向，这里先给一个常用符号
        cmd = int(AUTO_DEPTH_KP * dy)
        cmd = max(min(cmd, AUTO_DEPTH_FORCE_LIMIT), -AUTO_DEPTH_FORCE_LIMIT)

    global depth_force_filt
    depth_force_filt = DEPTH_ALPHA * cmd + (1.0 - DEPTH_ALPHA) * depth_force_filt
    return int(depth_force_filt)


# ========== 手柄初始化与模式切换 ==========
pygame.init()
pygame.joystick.init()
joystick = None

isHovering = False
auto_mode = False

last_planar_xy = (None, None)
last_yaw_rate = None
last_depth_force = None

if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("[JOYSTICK] Detected")
else:
    print("[JOYSTICK] Not detected")


# ========== 主程序 ==========
connect_to_pi()
print("Press s to select ROI, z to stop tracking, q to quit")
print("Manual: left stick planar, right stick yaw/depth")
print("Auto: planar + yaw + depth are controlled by program")

while True:
    frame = gst_read()
    if frame is None:
        continue

    h_img, w_img, _ = frame.shape
    CENTER_X = w_img / 2.0
    CENTER_Y = h_img / 2.0

    key = cv2.waitKey(1) & 0xFF

    if key == ord("z"):
        tracking = False
        tracking_paused = True
        width_history.clear()
        if isHovering:
            send_xy(0, 0)
            last_planar_xy = (0, 0)
        print("[TRACK] Stopped. Press S to reselect ROI.")

    if key == ord("s"):
        bbox = cv2.selectROI("Track", frame, False)
        tracker = cv2.legacy.TrackerCSRT_create()
        tracker.init(frame, bbox)
        tracking = True
        tracking_paused = False
        width_history.clear()
        print("[TRACK] Started")

    if key == ord("q"):
        break

    # ---------- 手柄事件 ----------
    for event in pygame.event.get():
        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == 7:
                send_pkt(PROPELLER_FRAME_HEAD, CMD_TOGGLE_FLOAT, b"")
                isHovering = not isHovering
                print("[HOVER]", "ON" if isHovering else "OFF")
                if not isHovering:
                    send_xy(0, 0)
                    last_planar_xy = (0, 0)

            elif event.button == 0:
                send_pkt(PROPELLER_FRAME_HEAD, CMD_TOGGLE_YAW_LOOP, b"")
                print("[YAW] Toggle yaw loop")

            elif event.button == 3:
                auto_mode = not auto_mode
                print("[MODE]", "AUTO" if auto_mode else "MANUAL")
                send_xy(0, 0)
                last_planar_xy = (0, 0)
                # 切换模式时重置滤波，避免冲击
                yaw_rate_filt = 0.0
                depth_force_filt = 0.0
                last_yaw_rate = None
                last_depth_force = None

    # ---------- 摇杆轴 ----------
    leftH = leftV = rightH = rightV = 0.0
    if joystick is not None:
        try:
            leftH = joystick.get_axis(0)
            leftV = joystick.get_axis(1)
            rightH = joystick.get_axis(3)
            rightV = joystick.get_axis(4)
        except Exception:
            pass

    # ========= 手动模式：左摇杆平面控制 =========
    if isHovering and (not auto_mode):
        x_pct = calculate_speed(leftH, DIRECTION_THRESHOLD)
        y_pct = -calculate_speed(leftV, DIRECTION_THRESHOLD)
        if (x_pct, y_pct) != last_planar_xy:
            send_xy(x_pct, y_pct)
            last_planar_xy = (x_pct, y_pct)

    # ========= 艏摇/上下：手动模式交给右摇杆；自动模式交给程序 =========
    if isHovering and (not auto_mode):
        # ---- 手动偏航角速度 ----
        yaw_rate_cmd = 0
        if not (-ROTATE_THRESHOLD < rightH < ROTATE_THRESHOLD):
            yaw_rate_cmd = calculate_speed(rightH, ROTATE_THRESHOLD)

        if yaw_rate_cmd != last_yaw_rate:
            send_pkt(PROPELLER_FRAME_HEAD, CMD_SET_YAW_RATE, i16le(yaw_rate_cmd))
            last_yaw_rate = yaw_rate_cmd

        # ---- 手动深度推力 ----
        depth_force_cmd = 0
        if not (-DEPTH_THRESHOLD < rightV < DEPTH_THRESHOLD):
            depth_force_cmd = calculate_speed(rightV, DEPTH_THRESHOLD)

        if depth_force_cmd != last_depth_force:
            send_pkt(PROPELLER_FRAME_HEAD, CMD_SET_DEPTH_FORCE, i16le(depth_force_cmd))
            last_depth_force = depth_force_cmd

    auto_x = 0
    auto_y = 0
    est_dist = None
    auto_yaw = 0
    auto_depth = 0

    # ========= 跟踪更新 + 自动控制 =========
    if tracking:
        ok, bbox = tracker.update(frame)
        if ok:
            x, y, bw, bh = bbox
            cx = x + bw / 2.0
            cy = y + bh / 2.0

            cv2.rectangle(frame, (int(x), int(y)),
                          (int(x + bw), int(y + bh)),
                          (0, 255, 0), 2)

            width_history.append(bw)
            w_smooth = sum(width_history) / len(width_history)

            if auto_mode and isHovering:
                now = time.time()
                if now - last_ctrl_time >= CTRL_INTERVAL:
                    last_ctrl_time = now

                    # 平面自动
                    auto_x, auto_y, est_dist = calc_planar_control(cx, w_smooth)
                    print("auto_x=", auto_x, "auto_y=", auto_y,)
                    send_xy_2(auto_x, auto_y)
                    last_planar_xy = (auto_x, auto_y)

                    # 艏摇自动
                    auto_yaw = calc_auto_yaw_rate(cx)
                    if auto_yaw != last_yaw_rate:
                        send_pkt(PROPELLER_FRAME_HEAD, CMD_SET_YAW_RATE, i16le(-auto_yaw))
                        last_yaw_rate = auto_yaw

                    ## 上下自动
                    auto_depth = calc_auto_depth_force(cy)
                    if auto_depth != last_depth_force:
                        send_pkt(PROPELLER_FRAME_HEAD, CMD_SET_DEPTH_FORCE, i16le(auto_depth))
                        last_depth_force = auto_depth
            else:
                w_smooth = max(float(w_smooth), 1.0)
                est_dist = TARGET_DIST * (W_AT_TARGET / w_smooth)

            mode_text = "AUTO" if auto_mode else "MANUAL"
            cv2.putText(frame,
                        f"Mode:{mode_text} x={auto_x} y={auto_y} yaw={auto_yaw} dep={auto_depth}",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            if est_dist is not None:
                cv2.putText(frame,
                            f"EstDist={est_dist:.2f}m",
                            (20, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            if auto_mode and isHovering:
                send_xy(0, 0)
                last_planar_xy = (0, 0)
                # 丢失目标时也把艏摇/上下停掉
                if last_yaw_rate != 0:
                    send_pkt(PROPELLER_FRAME_HEAD, CMD_SET_YAW_RATE, i16le(0))
                    last_yaw_rate = 0
                if last_depth_force != 0:
                    send_pkt(PROPELLER_FRAME_HEAD, CMD_SET_DEPTH_FORCE, i16le(0))
                    last_depth_force = 0

            width_history.clear()
            cv2.putText(frame, "Target Lost!",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow("Track", frame)

pipeline.set_state(Gst.State.NULL)
cv2.destroyAllWindows()
pygame.quit()
try:
    if sock:
        sock.close()
except Exception:
    pass
