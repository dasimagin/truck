import copy
import json
import socket

import psutil
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LAUNCH_PARAMS = {
    "lidar_ip": "192.168.2.10",
    "host_iface": "eth1",
    "frame_id": "livox",
    "publish_freq": "10",
    "scan_pattern_mode": "0",
}

NODE_PARAMS = {
    "frame_id": ...,  # from launch params
    "publish_freq": ...,  # from launch params
    "user_config_path": ...,  # generated in runtime
    "xfer_format": 0,  # 0: Pointcloud2(PointXYZRTL), 1: Custom format
    "multi_topic": 0,  # 0: all lidars same topic, 1: one lidar one topic
    "data_src": 0,  # keep default
    "output_data_type": 0,  # keep default
}

# https://github.com/Livox-SDK/livox_ros_driver2?tab=readme-ov-file#4-lidar-config
# pattern_mode (from launch params):
#   0: non-repeating scanning pattern mode
#   1: repeating scanning pattern mode
#   2: repeating scanning pattern mode (low scanning rate)
# pcl_data_type:
#   1: Cartesian coordinate data (32 bits)
#   2: Cartesian coordinate data (16 bits)
#   3: Spherical coordinate data
# do not touch:
#   host_ip: detected automatically
#   lidar_configs > ip: taken from launch params
#   lidar_type: 8 (no other options)
#   *_data_port: keep defaults
#   extrinsic_parameter: keep zeros because we're using transforms
LIVOX_CONFIG = {
    "lidar_summary_info": {"lidar_type": 8},
    "MID360": {
        "lidar_net_info": {
            "cmd_data_port": 56100,
            "push_msg_port": 56200,
            "point_data_port": 56300,
            "imu_data_port": 56400,
            "log_data_port": 56500,
        },
        "host_net_info": {
            "host_ip": ...,
            "cmd_data_port": 56101,
            "push_msg_port": 56201,
            "point_data_port": 56301,
            "imu_data_port": 56401,
            "log_data_port": 56501,
        },
    },
    "lidar_configs": [
        {
            "ip": ...,
            "pattern_mode": ...,
            "pcl_data_type": 1,
            "extrinsic_parameter": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "x": 0,
                "y": 0,
                "z": 0,
            },
        }
    ],
}


# https://stackoverflow.com/a/73559817
def get_ip_address(interface: str) -> str:
    interface_addrs = psutil.net_if_addrs().get(interface) or []
    for snicaddr in interface_addrs:
        if snicaddr.family == socket.AF_INET:
            return snicaddr.address


# https://robotics.stackexchange.com/a/104402
def launch_setup(context):
    launch_params = {x: LaunchConfiguration(x).perform(context) for x in LAUNCH_PARAMS}
    lidar_ip = launch_params["lidar_ip"]
    host_iface = launch_params["host_iface"]
    host_ip = get_ip_address("eth1")
    assert host_ip is not None, f"Failed to get host IP for interface {host_iface}"

    config = copy.deepcopy(LIVOX_CONFIG)
    config["MID360"]["host_net_info"]["host_ip"] = host_ip
    config["lidar_configs"][0]["ip"] = lidar_ip
    config["lidar_configs"][0]["pattern_mode"] = int(launch_params["scan_pattern_mode"])
    config_path = "/tmp/livox-config.json"
    with open(config_path, "w") as file:
        json.dump(config, file, sort_keys=False)

    print(f"[livox launch] Lidar IP: {lidar_ip}")
    print(f"[livox launch] Host IP: {host_ip} ({host_iface})")
    print(f"[livox launch] Config path: {config_path}")

    node_params = copy.deepcopy(NODE_PARAMS)
    node_params["frame_id"] = launch_params["frame_id"]
    node_params["publish_freq"] = float(launch_params["publish_freq"])
    node_params["user_config_path"] = config_path
    node_params = [{name: value} for name, value in node_params.items()]

    node = Node(
        package="livox_driver",
        name="livox_driver",
        executable="livox_driver_node",
        output="screen",
        parameters=node_params,
    )
    return [node]


def generate_launch_description():
    launch_params = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in LAUNCH_PARAMS.items()
    ]
    description = LaunchDescription(launch_params)
    description.add_action(OpaqueFunction(function=launch_setup))
    return description
