"""Jetson side of navigation: lidar mount transform + two flat scans.

/scan is cut low: what the robot could walk into, for Nav2's costmaps.
/scan_walls is cut above the furniture: the walls a floor map shows, for
AMCL. Measured 2026-10-09 in the lab: of the low scan 45% lands on mapped
walls, of the high one 87%.

This file runs on the Orin as ~/bin/lidar_nav.launch.py (lidar-scan.service);
the copy in the robot repo's env/ is for the record.

    ros2 launch ~/bin/lidar_nav.launch.py
    ros2 launch ~/bin/lidar_nav.launch.py lidar_z:=0.12 lidar_pitch:=0.356

Runs next to the Livox driver. Nav2 itself runs on the robot PC (Foxy) and
picks /scan and /tf_static up over DDS (same ROS_DOMAIN_ID, same LAN).

Mount: measured 20.4 deg nose-down from the MID360's own accelerometer
(2026-09-24). Nose-down is POSITIVE pitch about y under REP-103: the sensor x
axis maps to (cos p, 0, -sin p). The x/y/z offset from base_link (body centre)
is not measured yet - the defaults are guesses; measure and pass them in.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node

ARGS = {  # name: (default, meaning)
    "lidar_x": ("0.20", "m forward of base_link"),
    "lidar_y": ("0.0", "m left of base_link"),
    "lidar_z": ("0.10", "m above base_link"),
    "lidar_pitch": ("0.356", "rad nose-down (20.4 deg)"),
    "scan_min_z": ("-0.15", "m in base_link: lowest obstacle slice for /scan"),
    "scan_max_z": ("1.0", "m in base_link: highest"),
    "walls_min_z": ("0.9", "m in base_link: /scan_walls from here (base_link stands 0.32 m over the floor)"),
    "walls_max_z": ("1.7", "m in base_link: to here; 0.2 m higher already caught something hanging in the lab"),
}


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v, description=d)
         for k, (v, d) in ARGS.items()] + [
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="tf_base_link_livox",
             arguments=["--x", L("lidar_x"), "--y", L("lidar_y"),
                        "--z", L("lidar_z"), "--pitch", L("lidar_pitch"),
                        "--frame-id", "base_link", "--child-frame-id", "livox_frame"]),
        # 3D cloud -> 2D scan in base_link, so the 20 deg tilt is removed and
        # AMCL sees walls as walls. Ranges under 0.4 m are the robot itself.
        Node(package="pointcloud_to_laserscan",
             executable="pointcloud_to_laserscan_node", name="lidar_to_scan",
             remappings=[("cloud_in", "/livox/lidar"), ("scan", "/scan")],
             parameters=[{
                 "target_frame": "base_link",
                 "transform_tolerance": 0.05,
                 "min_height": L("scan_min_z"),
                 "max_height": L("scan_max_z"),
                 "angle_min": -3.14159, "angle_max": 3.14159,
                 "angle_increment": 0.00873,     # 0.5 deg
                 "scan_time": 0.1,
                 "range_min": 0.4, "range_max": 30.0,
                 "use_inf": True,
             }]),
        Node(package="pointcloud_to_laserscan",
             executable="pointcloud_to_laserscan_node", name="lidar_to_scan_walls",
             remappings=[("cloud_in", "/livox/lidar"), ("scan", "/scan_walls")],
             parameters=[{
                 "target_frame": "base_link",
                 "transform_tolerance": 0.05,
                 "min_height": L("walls_min_z"),
                 "max_height": L("walls_max_z"),
                 "angle_min": -3.14159, "angle_max": 3.14159,
                 "angle_increment": 0.00873,     # 0.5 deg
                 "scan_time": 0.1,
                 "range_min": 0.4, "range_max": 30.0,
                 "use_inf": True,
             }]),
    ])
