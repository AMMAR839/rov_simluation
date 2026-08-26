#!/usr/bin/env python3
"""Capture Gazebo maneuver GIF from an underwater camera perspective.

Moves the Gazebo GUI camera to an underwater position near the ROV,
then records maneuvers via screen-grab.  No extra in-SDF camera sensors
are added, so simulation real-time factor stays at ~99%.
"""
import os
import time
import subprocess
import ctypes
import threading
from PIL import Image
import mss

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String

# ---------- ROV spawn & camera config ----------
ROV_X, ROV_Y, ROV_Z = -3.20, 0.0, -0.28

# Underwater camera: ~2.5 m away, slightly below the ROV
# Position: behind-right of the ROV, submerged
CAM_X, CAM_Y, CAM_Z = -1.20, -1.80, -0.35

# Orientation quaternion pointing camera toward the ROV
# yaw ≈ 2.38 rad (≈136°), pitch ≈ 0.03 rad (slight up-tilt)
# Computed from look-at direction (-2.0, 1.8, 0.07)
CAM_QX, CAM_QY, CAM_QZ, CAM_QW = 0.0, 0.015, 0.929, 0.370


def move_gui_camera(x, y, z, qx, qy, qz, qw):
    """Move Gazebo GUI camera via gz service."""
    req = (
        f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}}}'
    )
    cmd = [
        'gz', 'service', '-s', '/gui/move_to/pose',
        '--reqtype', 'gz.msgs.GUICamera',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '3000',
        '--req', req,
    ]
    result = subprocess.run(cmd, capture_output=True, timeout=10)
    ok = result.returncode == 0
    if ok:
        print(f"  GUI camera moved to ({x}, {y}, {z})")
    else:
        # Fallback: try via /gui/move_to service with Pose msg
        print(f"  move_to/pose failed ({result.returncode}), trying /gui/camera/pose...")
        cmd2 = [
            'gz', 'service', '-s', '/gui/camera/pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', f'position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}',
        ]
        r2 = subprocess.run(cmd2, capture_output=True, timeout=10)
        if r2.returncode == 0:
            print(f"  GUI camera moved (fallback) to ({x}, {y}, {z})")
        else:
            print(f"  WARNING: Could not move GUI camera. Stderr: {r2.stderr.decode()[:200]}")
    return ok


def bring_gazebo_to_front():
    x11 = ctypes.cdll.LoadLibrary('libX11.so.6')
    display = x11.XOpenDisplay(None)
    if not display:
        return None

    out = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True).stdout.decode()
    gz_win = None
    for line in out.splitlines():
        if '0x' in line:
            line_l = line.lower()
            if 'chrome' in line_l or 'youtube' in line_l or 'antigravity' in line_l or 'code' in line_l:
                win_id = int(line.strip().split()[0], 16)
                x11.XIconifyWindow(display, ctypes.c_ulong(win_id), 0)
            elif 'gazebo sim' in line_l or 'gz-sim-gui' in line_l:
                parts = line.strip().split()
                gz_win = int(parts[0], 16)

    if gz_win:
        x11.XMapRaised(display, ctypes.c_ulong(gz_win))
        x11.XFlush(display)
    x11.XCloseDisplay(display)
    return gz_win


def capture_gazebo_viewport_bbox():
    out = subprocess.run(['xwininfo', '-name', 'Gazebo Sim'], capture_output=True).stdout.decode()
    x, y, w, h = 102, 20, 1200, 700
    for line in out.splitlines():
        line = line.strip()
        if line.startswith('Absolute upper-left X:'):
            x = int(line.split(':')[1])
        elif line.startswith('Absolute upper-left Y:'):
            y = int(line.split(':')[1])
        elif line.startswith('Width:'):
            w = int(line.split(':')[1])
        elif line.startswith('Height:'):
            h = int(line.split(':')[1])

    vx = x + 12
    vy = y + 85
    vw = max(400, w - 345)
    vh = max(300, h - 120)
    return {'top': vy, 'left': vx, 'width': vw, 'height': vh}


class ManeuverRecorder(Node):
    def __init__(self):
        super().__init__('maneuver_recorder')
        self.cmd_manual_pub = self.create_publisher(Twist, '/rov/manual_cmd_vel', 10)
        self.cmd_direct_pub = self.create_publisher(Twist, '/rov/cmd_vel', 10)
        self.source_pub = self.create_publisher(String, '/rov/command_source', 10)
        self.gripper_pub = self.create_publisher(Float64, '/rov/gripper_cmd', 10)

        self.target_twist = Twist()
        self.current_gripper = 0.0
        self.running = True

        self.pub_thread = threading.Thread(target=self._loop_publish, daemon=True)
        self.pub_thread.start()

    def _loop_publish(self):
        while self.running and rclpy.ok():
            self.source_pub.publish(String(data="manual"))
            self.cmd_manual_pub.publish(self.target_twist)
            self.cmd_direct_pub.publish(self.target_twist)
            self.gripper_pub.publish(Float64(data=self.current_gripper))
            time.sleep(0.02)

    def set_twist(self, lx=0.0, ly=0.0, lz=0.0, az=0.0):
        t = Twist()
        t.linear.x = float(lx)
        t.linear.y = float(ly)
        t.linear.z = float(lz)
        t.angular.z = float(az)
        self.target_twist = t

    def set_gripper(self, val):
        self.current_gripper = float(val)

    def stop(self):
        self.target_twist = Twist()

    @staticmethod
    def grab_frame(sct, bbox):
        raw = sct.grab(bbox)
        return Image.frombytes('RGB', raw.size, raw.bgra, 'raw', 'BGRX')


def main():
    print("Starting Underwater Gazebo Recorder...")
    rclpy.init()
    node = ManeuverRecorder()

    bring_gazebo_to_front()
    time.sleep(2.5)

    # Move GUI camera to underwater position
    print("Moving GUI camera to underwater position...")
    move_gui_camera(CAM_X, CAM_Y, CAM_Z, CAM_QX, CAM_QY, CAM_QZ, CAM_QW)
    time.sleep(1.5)  # Let render settle

    bbox = capture_gazebo_viewport_bbox()
    os.makedirs('/home/ammar/Documents/WS_ROV/docs/images', exist_ok=True)

    with mss.mss() as sct:
        # --- Photo 1: Underwater close-up ---
        print("Capturing Photo 1 (Underwater close-up)...")
        img1 = node.grab_frame(sct, bbox)
        img1.save('/home/ammar/Documents/WS_ROV/docs/images/gazebo_rov_closeup.png')
        print("  Saved gazebo_rov_closeup.png")

        # --- Photo 2: Slightly different angle ---
        # Move camera to a second underwater angle (front-left)
        move_gui_camera(-4.60, 1.20, -0.32, 0.0, -0.01, -0.37, 0.93)
        time.sleep(1.0)
        print("Capturing Photo 2 (Underwater front-left)...")
        img2 = node.grab_frame(sct, bbox)
        img2.save('/home/ammar/Documents/WS_ROV/docs/images/gazebo_overview.png')
        print("  Saved gazebo_overview.png")

        # --- Move back to main underwater angle for GIF ---
        move_gui_camera(CAM_X, CAM_Y, CAM_Z, CAM_QX, CAM_QY, CAM_QZ, CAM_QW)
        time.sleep(1.0)

        # --- Single Combined Maneuver GIF ---
        print("Recording Combined Underwater Maneuver GIF (extended displacement)...")
        all_frames = []

        # Phase 1: Vertical Heave — dive deep then ascend back
        print("  Phase 1: Vertical Heave (extended)...")
        for i in range(35):
            spd = -0.55 * min(1.0, (i + 1) / 5.0)
            node.set_twist(lz=spd)
            frame = node.grab_frame(sct, bbox).resize((720, 480), Image.Resampling.LANCZOS)
            all_frames.append(frame)
            time.sleep(0.07)

        for i in range(35):
            spd = 0.55 * min(1.0, (i + 1) / 5.0)
            node.set_twist(lz=spd)
            frame = node.grab_frame(sct, bbox).resize((720, 480), Image.Resampling.LANCZOS)
            all_frames.append(frame)
            time.sleep(0.07)

        node.stop()
        time.sleep(0.2)

        # Phase 2: Sideways Sway — long strafe left then right
        print("  Phase 2: Sideways Sway (extended)...")
        for i in range(30):
            spd = 0.55 * min(1.0, (i + 1) / 5.0)
            node.set_twist(ly=spd)
            frame = node.grab_frame(sct, bbox).resize((720, 480), Image.Resampling.LANCZOS)
            all_frames.append(frame)
            time.sleep(0.07)

        for i in range(30):
            spd = -0.55 * min(1.0, (i + 1) / 5.0)
            node.set_twist(ly=spd)
            frame = node.grab_frame(sct, bbox).resize((720, 480), Image.Resampling.LANCZOS)
            all_frames.append(frame)
            time.sleep(0.07)

        node.stop()
        time.sleep(0.2)

        # Phase 3: Claw/Gripper open-close
        print("  Phase 3: Claw Movement...")
        for _ in range(20):
            node.set_gripper(1.0)
            frame = node.grab_frame(sct, bbox).resize((720, 480), Image.Resampling.LANCZOS)
            all_frames.append(frame)
            time.sleep(0.07)

        for _ in range(20):
            node.set_gripper(0.0)
            frame = node.grab_frame(sct, bbox).resize((720, 480), Image.Resampling.LANCZOS)
            all_frames.append(frame)
            time.sleep(0.07)

        # Phase 4: Forward surge + Yaw turning — wide arc
        print("  Phase 4: Dynamic Yaw & Surge (extended)...")
        for _ in range(28):
            node.set_twist(lx=0.45, az=0.30)
            frame = node.grab_frame(sct, bbox).resize((720, 480), Image.Resampling.LANCZOS)
            all_frames.append(frame)
            time.sleep(0.07)

        for _ in range(28):
            node.set_twist(lx=-0.45, az=-0.30)
            frame = node.grab_frame(sct, bbox).resize((720, 480), Image.Resampling.LANCZOS)
            all_frames.append(frame)
            time.sleep(0.07)

        node.stop()

        if all_frames:
            gif_path = '/home/ammar/Documents/WS_ROV/docs/images/gazebo_maneuver_all.gif'
            all_frames[0].save(gif_path, save_all=True, append_images=all_frames[1:], duration=70, loop=0)
            print(f"Saved maneuver GIF: {gif_path} ({len(all_frames)} frames)")

    node.running = False
    node.destroy_node()
    rclpy.shutdown()
    print("Underwater Recording Completed!")


if __name__ == "__main__":
    main()
