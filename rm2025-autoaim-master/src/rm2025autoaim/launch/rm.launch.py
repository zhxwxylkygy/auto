import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
import launch
from launch.actions import TimerAction, Shutdown
launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rm2025autoaim'), 'config', 'launch_params.yaml')))

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
                    extra_arguments=[{'use_intra_process_comms': True}]
                    )
    
    container = ComposableNodeContainer(
            name='img_process_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
                sync_node,
                detect_node,
                est_node,
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                            'frame_processer:='+launch_params['frame_processer_log_level']],
            respawn = True,
            respawn_delay = 2.0,
    )
    

    mcu_node = Node(
    package='mcu_node',
    executable='mcu_node',
    output='both',
    respawn = True,
    respawn_delay = 1.0,
    emulate_tty=True,
    #parameters=[node_params],
    ros_arguments=['--log-level', 'mcu_node:='+launch_params['mcu_node_log_level']],
    )
    task_manager = Node(
    package='task_manager',
    executable='task_manager',
    output='both',
    respawn = True,
    respawn_delay = 0.1,
    emulate_tty=True,
    #parameters=[node_params],
    ros_arguments=['--log-level', 'mcu_node:='+launch_params['task_manager_log_level']],
    )
    delay_serial_node = TimerAction(
        period=0.5,
        actions=[mcu_node],
    )

    delay_image_process_container = TimerAction(
        period=1.0,
        actions=[container],
    )
    delay_task_manager = TimerAction(
        period=2.0,
        actions=[task_manager],
    )
    return launch.LaunchDescription([delay_serial_node,
                                     delay_task_manager,
                                     delay_image_process_container,
                                     ])