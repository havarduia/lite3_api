"""rs_stream - the RealSense colour image as H.264 on the robot's mediamtx.

    python3 -m robot.rs_stream [topic]      # another topic, for testing

Pipes the camera's JPEG topic through GStreamer and the Jetson's hardware
encoder to RS_RTSP. A process of its own (hmi.py starts it); exits when
that one does. Why JPEG, hardware and a separate process: README.md 10.3.
"""
import os
import subprocess
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from .protocol import RS_RTSP

TOPIC = '/camera/color/image_raw/compressed'
FPS = 30                # what the camera is set to (env/camera.launch.py)
BITRATE = 3000000       # about what the front camera sends at the same size


class Stream:
    def __init__(self):
        self.gst = None

    def frame(self, m):
        if self.gst is None or self.gst.poll() is not None:
            self.close()
            self.gst = subprocess.Popen(
                ['gst-launch-1.0', '-q', 'fdsrc', 'fd=0', 'do-timestamp=true', '!',
                 'image/jpeg,framerate=%d/1' % FPS, '!', 'jpegparse', '!',
                 # jpegdec, not nvjpegdec: the hardware JPEG decoder kept handing
                 # out its first frame, so the video was a still (2026-10-05).
                 'jpegdec', '!', 'video/x-raw,format=I420', '!', 'nvvidconv', '!',
                 'video/x-raw(memory:NVMM),format=NV12', '!', 'nvv4l2h264enc',
                 'bitrate=%d' % BITRATE,
                 # baseline: no B-frames, which WebRTC cannot take; a keyframe
                 # a second so a new viewer starts at once.
                 'profile=0', 'insert-sps-pps=true', 'iframeinterval=%d' % FPS,
                 'idrinterval=%d' % FPS, '!', 'h264parse', '!', 'rtspclientsink',
                 'location=' + RS_RTSP, 'protocols=tcp', 'latency=0'],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL)
        try:
            self.gst.stdin.write(m.data)
            self.gst.stdin.flush()
        except OSError:                     # gst died (mediamtx restarting?)
            self.close()
            time.sleep(1.0)

    def close(self):
        if self.gst:
            self.gst.kill()
            self.gst.wait()
            self.gst = None


def main():
    parent = os.getppid()
    rclpy.init()
    node = rclpy.create_node('rs_stream')
    stream = Stream()
    node.create_subscription(CompressedImage, sys.argv[1] if len(sys.argv) > 1 else TOPIC,
                             stream.frame, qos_profile_sensor_data)
    try:
        while os.getppid() == parent:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stream.close()


if __name__ == '__main__':
    main()
