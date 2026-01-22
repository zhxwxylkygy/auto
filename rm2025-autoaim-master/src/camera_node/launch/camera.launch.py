from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_node',  # 你的包名
            #namespace='empty',        # 可选，命名空间
            executable='camera_node',     # 你的节点可执行文件名（不包括.py扩展名）
            name='imgpublisher',                # 节点名
            output='screen',              # 输出到屏幕
            cwd='/root/master/program/RM2025--autoaim'
            # parameters=[
            #     {'param_name': 'param_value'}  # 可选，参数列表
            # ],
            # remappings=[
            #     ('/input_topic', '/output_topic')  # 可选，话题重映射
            # ]
        )
    ])