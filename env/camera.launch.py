"""The D435i on realsense-ros 4.58.3 + librealsense 2.58.4, patched kernel.

Drop-in for the vendor's dr_camera_launch.py (realsense-ros 3.2.3 fork): same
topic names (/camera/depth/color/points, /camera/color/image_raw, ...), same
frames (camera_link, camera_depth_optical_frame), and the vendor's two changes
to the cloud, which voa_ros2, Nav2 and depth.py all depend on:

  steady_stamp     the cloud is stamped with the steady clock, like every
                   other robot topic - otherwise TF lookups against it fail
  voxel_leaf_size  thinned to one point per 5 cm cube (vendor filter_voxel)

Both are our patch to pointcloud_filter.cpp in ~/realsense_ros4_ws; stock
4.58.3 ignores them.

Unlike the vendor launch this also streams colour and the camera's own IMU:
with the patched uvcvideo, depth + colour at 30 fps no longer takes the USB
bus down (verified 2026-09-24). Colour stays 424x240 - there is no compressed
transport, and raw 640x480 at 30 is ~27 MB/s, hopeless over wifi.

    ros2 launch ~/robot/env/camera.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        # 4.x publishes on ~/..., so an empty namespace + name 'camera' gives
        # /camera/depth/... exactly as the vendor's 3.2.3 did.
        namespace='',
        name='camera',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'camera_name': 'camera',
            'enable_depth': True,
            'depth_module.depth_profile': '424,240,30',    # vendor's resolution
            'enable_color': True,
            'rgb_camera.color_profile': '424,240,30',
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': True,
            'enable_accel': True,
            'gyro_fps': 200,
            'accel_fps': 100,
            'unite_imu_method': 2,              # linear interpolation -> /camera/imu
            'pointcloud__neon_.enable': True,        # ARM builds name the filter pointcloud__neon_
            'pointcloud__neon_.stream_filter': 0,     # XYZ only, no colour texture
            'pointcloud.steady_stamp': True,
            'pointcloud.voxel_leaf_size': 0.05,
            # Cut depth past 5 m before the voxel grid: far points make PCL's
            # 5 cm grid overflow its index, and it then silently passes the whole
            # raw cloud through. Nav2 marks obstacles to 3 m, depth.py looks to 4.
            'clip_distance': 5.0,
            'align_depth.enable': False,
            'initial_reset': False,
        }],
    )])
