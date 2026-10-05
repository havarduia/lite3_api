"""rs_stream - the RealSense colour image as H.264 on the robot's mediamtx.

    python3 -m robot.rs_stream

Subscribes to /camera/color/image_raw and pipes the frames through ffmpeg
(libx264) to RS_RTSP, so the HMI page can show it over WebRTC exactly like
the front camera. A process of its own on purpose (hmi.py starts it): encoded
inside the HMI it stuttered, 22 fps with 150-200 ms gaps against a steady
29 fps here (measured 2026-10-05). Exits when the process that started it does.

This is sized for the camera's 424x240. Raw 1280x720 frames do not get
through ROS into Python fast enough here: a subscriber that does nothing
received 8 of 30 frames a second (measured 2026-10-05). About 640x360 is the
ceiling for this route; beyond that the image has to leave the camera driver
already compressed (compressed_image_transport, not installed).
"""
import os
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .protocol import RS_RTSP

FPS = 30                # what the camera is set to (env/camera.launch.py)
PIX = {'rgb8': 'rgb24', 'bgr8': 'bgr24'}


class Stream:
    def __init__(self):
        self.ff, self.shape = None, None

    def frame(self, m):
        shape = (m.encoding, m.width, m.height)
        if m.encoding not in PIX or m.step != m.width * 3:
            return
        if self.ff is None or self.ff.poll() is not None or shape != self.shape:
            self.close()
            self.shape = shape
            self.ff = subprocess.Popen(
                ['ffmpeg', '-loglevel', 'error', '-f', 'rawvideo',
                 '-pix_fmt', PIX[m.encoding], '-s', '%dx%d' % (m.width, m.height),
                 '-framerate', str(FPS), '-i', '-',
                 # baseline + zerolatency: no B-frames, which WebRTC cannot
                 # take; a keyframe a second so a new viewer starts at once.
                 '-c:v', 'libx264', '-preset', 'ultrafast', '-tune', 'zerolatency',
                 '-profile:v', 'baseline', '-pix_fmt', 'yuv420p', '-g', str(FPS),
                 '-f', 'rtsp', '-rtsp_transport', 'tcp', RS_RTSP],
                stdin=subprocess.PIPE)
        try:
            self.ff.stdin.write(m.data)
        except OSError:                     # ffmpeg died (mediamtx restarting?)
            self.close()
            time.sleep(1.0)

    def close(self):
        if self.ff:
            self.ff.kill()
            self.ff.wait()
            self.ff = None


def main():
    parent = os.getppid()
    rclpy.init()
    node = rclpy.create_node('rs_stream')
    stream = Stream()
    node.create_subscription(Image, '/camera/color/image_raw', stream.frame,
                             qos_profile_sensor_data)
    try:
        while os.getppid() == parent:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stream.close()


if __name__ == '__main__':
    main()
