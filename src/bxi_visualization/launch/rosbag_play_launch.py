import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess,Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 修bug
    os.environ['QT_ENABLE_HIGHDPI_SCALING'] = '0'

    # config
    urdf_path = os.path.join(
        get_package_share_directory("bxi_visualization"),
        "model/urdf/bot_elf/urdf/bot_elf.urdf",
    )
    urdf = open(urdf_path).read()
    rviz_config = os.path.join(
        get_package_share_directory("bxi_visualization"),
        "config/bot_elf_config.rviz",
    )

    
    topic_prefix_arg = DeclareLaunchArgument(
                    "topic_prefix",
                    default_value='None',
                    description='topic prefix of ros bag'
                )
    sim_time_arg = DeclareLaunchArgument(
                    "use_sim_time",
                    default_value="true",
                    description="Use simulation (Gazebo) clock if true",
                )

    use_sim_time = LaunchConfiguration("use_sim_time")
    topic_prefix = LaunchConfiguration("topic_prefix")

    rosbag_play_process = Node(
                    package='bxi_visualization',
                    executable='rosbag_controller.py',
                    name='rosbag_controller',
                    output='screen',
                    parameters=[
                    {"/topic_prefix": topic_prefix},
                    ],
                    on_exit=Shutdown(),
    )
    odom2tf_node = Node(
                    package='bxi_visualization',
                    executable='odom2tf',
                    name='odom2tf',
                    output='screen',
            )

    rivz_node = Node(   
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                on_exit=Shutdown(),
            )
    robot_state_publisher_node = Node(   
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"robot_description": urdf},
                    {"publish_frequency": 1000.0},
                ],
            )

    # 返回 LaunchDescription，包含参数声明和 rosbag 播放节点
    return LaunchDescription([
        # arg
        topic_prefix_arg,
        sim_time_arg,
        # node
        rosbag_play_process,
        odom2tf_node,
        rivz_node,
        robot_state_publisher_node
    ])