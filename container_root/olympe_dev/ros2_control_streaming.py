#!/usr/bin/env python3

import csv
import math
import os
import queue
import shlex
import subprocess
import tempfile
import threading
import time
import cv2
import numpy as np
import datetime
from pynput.keyboard import Key, Listener

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.PilotingSettingsState import MaxTiltChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.video.renderer import PdrawRenderer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

DRONE_IP = "192.168.42.1"
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")


class StreamingExample(Node):
    def __init__(self):
        super().__init__('drone_image_publisher')
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.drone = olympe.Drone(DRONE_IP)
        current_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        file = tempfile.mkdtemp(prefix=f"olympe_streaming_{current_time}_", dir="./")
        self.output_dir = os.path.abspath(file)
        self.get_logger().info(f"Olympe streaming example output dir: {self.output_dir}")
        self.h264_frame_stats = []
        self.h264_stats_file = open(os.path.join(self.output_dir, "h264_stats.csv"), "w+")
        self.h264_stats_writer = csv.DictWriter(self.h264_stats_file, ["fps", "bitrate"])
        self.h264_stats_writer.writeheader()
        self.frame_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self.yuv_frame_processing)
        self.renderer = None
        self.keyboard_listener = Listener(on_press=self.on_key_press)
        self.keyboard_listener.start()

    def start(self):
        assert self.drone.connect(retry=3)
        if DRONE_RTSP_PORT is not None:
            self.drone.streaming.server_addr = f"{DRONE_IP}:{DRONE_RTSP_PORT}"
        self.drone.streaming.set_output_files(metadata=os.path.join(self.output_dir, "streaming_metadata.json"))
        self.drone.streaming.set_callbacks(raw_cb=self.yuv_frame_cb, h264_cb=self.h264_frame_cb, start_cb=self.start_cb, end_cb=self.end_cb, flush_raw_cb=self.flush_cb)
        self.drone.streaming.start()
        self.renderer = PdrawRenderer(pdraw=self.drone.streaming)
        self.running = True
        self.processing_thread.start()

    def stop(self):
        self.running = False
        self.processing_thread.join()
        if self.renderer is not None:
            self.renderer.stop()
        assert self.drone.streaming.stop()
        assert self.drone.disconnect()
        self.h264_stats_file.close()
        self.keyboard_listener.stop()

    def on_key_press(self, key):
        try:
            if key.char == 'w':
                self.drone(moveBy(1, 0, 0, 0, _timeout=5)).wait()
            elif key.char == 's':
                self.drone(moveBy(-1, 0, 0, 0, _timeout=5)).wait()
            elif key.char == 'a':
                self.drone(moveBy(0, -1, 0, 0, _timeout=5)).wait()
            elif key.char == 'd':
                self.drone(moveBy(0, 1, 0, 0, _timeout=5)).wait()
        except AttributeError:
            if key == Key.up:
                self.drone(moveBy(0, 0, -0.5, 0, _timeout=5)).wait()
            elif key == Key.down:
                self.drone(moveBy(0, 0, 0.5, 0, _timeout=5)).wait()

    def yuv_frame_cb(self, yuv_frame):
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)

    def yuv_frame_processing(self):
        while self.running:
            try:
                yuv_frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            self.save_yuv_frame_as_image(yuv_frame)
            yuv_frame.unref()

    def flush_cb(self, stream):
        if stream["vdef_format"] != olympe.VDEF_I420:
            return True
        while not self.frame_queue.empty():
            self.frame_queue.get_nowait().unref()
        return True

    def start_cb(self):
        pass

    def end_cb(self):
        pass

    def h264_frame_cb(self, h264_frame):
        frame_pointer, frame_size = h264_frame.as_ctypes_pointer()
        info = h264_frame.info()
        frame_ts = info["ntp_raw_timestamp"]
        if not bool(info["is_sync"]):
            while len(self.h264_frame_stats) > 0:
                start_ts, _ = self.h264_frame_stats[0]
                if (start_ts + 1e6) < frame_ts:
                    self.h264_frame_stats.pop(0)
                else:
                    break
            self.h264_frame_stats.append((frame_ts, frame_size))
            h264_fps = len(self.h264_frame_stats)
            h264_bitrate = 8 * sum(map(lambda t: t[1], self.h264_frame_stats))
            self.h264_stats_writer.writerow({"fps": h264_fps, "bitrate": h264_bitrate})

    def save_yuv_frame_as_image(self, yuv_frame):
        info = yuv_frame.info()
        yuv_array = np.frombuffer(yuv_frame.as_ndarray(), dtype=np.uint8)
        height, width = info["raw"]["frame"]["info"]["height"], info["raw"]["frame"]["info"]["width"]
        yuv_array = yuv_array.reshape((height * 3 // 2, width))
        bgr_frame = cv2.cvtColor(yuv_array, cv2.COLOR_YUV2BGR_I420)
        timestamp = int(time.time() * 1000)
        image_filename = os.path.join(self.output_dir, f"{timestamp}.jpg")
        cv2.imwrite(image_filename, bgr_frame)
        ros_image = self.bridge.cv2_to_imgmsg(bgr_frame, encoding="bgr8")
        self.image_pub.publish(ros_image)

    def main(args=None):
        rclpy.init(args=args)
        streaming_example = StreamingExample()
        streaming_example.start()
        try:
            rclpy.spin(streaming_example)
        except KeyboardInterrupt:
            pass
        finally:
            streaming_example.stop()
            streaming_example.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
