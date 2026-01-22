import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
import launch
from launch.actions import TimerAction, Shutdown


def generate_launch_description():
    camera_node  =  ComposableNode(
                    package='camera_node',
                    plugin='gxcamera::CameraNode',
                    name='camera_node', 
                    extra_arguments=[{'use_intra_process_comms': True}])
    sync_node = ComposableNode(
                    package='sensor_sync_node',
                    plugin='sensorsync::SensorSyncNode',
                    name='sensor_sync_node', 
                    extra_arguments=[{'use_intra_process_comms': True}])
    detect_node = ComposableNode(
                    package='detect_node',
                    plugin='detector::Subscriber',
                    name='detector_node', 
                    extra_arguments=[{'use_intra_process_comms': True}])
    est_node = ComposableNode(
                    package='estimate_node',
                    plugin='trackers::FeatureSubscriber',
                    name='estimate_node', 
                    extra_arguments=[{'use_intra_process_comms': True}])
    mcu_node = ComposableNode(
                    package='mcu_node',
                    plugin='mcunode::McuNode',
                    name='mcu_node', 
                    extra_arguments=[{'use_intra_process_comms': True}])
    
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                sync_node,
                camera_node,
                detect_node,
                est_node,          
            ],
            output='both',
            emulate_tty=True,
    )
    
    return launch.LaunchDescription([container])

