"""The D435i on realsense-ros 4.58.3: a drop-in for the vendor's
dr_camera_launch.py with the same topics and frames, plus colour and the
camera's IMU. steady_stamp and voxel_leaf_size are our patch to the driver.
README.md section 11.3.

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
            # Colour is only looked at (HMI, through rs_stream.py), which switches it
            # on while it runs: off saves the driver 6% of a core. So it can be
            # sharp; depth stays at the vendor's size, which Nav2 and voa expect.
            'enable_color': False,
            'rgb_camera.color_profile': '1280,720,30',
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
