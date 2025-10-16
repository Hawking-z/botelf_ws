import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import TimerAction,Shutdown,ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import datetime


def generate_launch_description():

    # 声明一个命令行参数 --log，默认值为 "false"
    log_arg = DeclareLaunchArgument(
        'log', default_value='false', description='Enable logging (ros2 bag recording)'
    )

    controller_arg = DeclareLaunchArgument(
        'controller',default_value='rl_controller',description='Controller type: rl_controller'
    )

    topic_prefix = "hardware"

     # 定义需要录制的主题列表
    topics_to_record = [
        topic_prefix+"/actuators_cmds",
        topic_prefix+"/imu_data",
        topic_prefix+"/touch_sensor",
        topic_prefix+"/joint_states",
        topic_prefix+"/action_output",
        "/motion_commands",
    ]
    
    # 自动生成唯一的保存路径（带时间戳）
    timestamp = datetime.now().strftime('%Y%m%d_%H%M')
    save_path = f'rosbag/rosbag2_{timestamp}_'+topic_prefix

    # 构建 ros2 bag record 命令，录制 /simulation/ 下的所有数据
    record_command = [
        'ros2', 'bag', 'record', '-o', save_path
    ] + topics_to_record
    # 获取参数值
    log_enabled = LaunchConfiguration('log')
    controller = LaunchConfiguration('controller')
    
    # 只有当 --log=true 时，才启动 ros2 bag record
    record_process = ExecuteProcess(
        cmd=record_command,
        output='screen',
        name='ros2_bag_record',
        condition=IfCondition(log_enabled)  # 条件判断
    )

    hardware_node = Node(
        package="hardware",
        executable="hardware",
        name="hardware",
        output="screen",
        parameters=[
        ],
        emulate_tty=True,
        arguments=[("__log_level:=debug")],
    )

    bxi_controller_node = Node(
        package="bxi_controller",
        executable=controller,
        name="bxi_controller",
        output="screen",
        parameters=[
            {"/topic_prefix": topic_prefix+"/"},
        ],
        emulate_tty=True,
        arguments=[("__log_level:=debug")],
    )

    return LaunchDescription(
        [
            controller_arg,
            log_arg,
            record_process,
            hardware_node,
             TimerAction(
                period=10.,  # 延迟时间（秒）
                actions=[bxi_controller_node],
            ),
        ]
    )
