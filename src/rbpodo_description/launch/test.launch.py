from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 패키지 및 경로
    pkg_name = 'rbpodo_description'
    pkg_share = get_package_share_directory(pkg_name)

    # URDF (xacro)
    xacro_file = os.path.join(pkg_share, 'robots', 'urdf', 'rb5_850e.urdf.xacro')
    robot_description = {
        'robot_description': Command(['xacro ', xacro_file])
    }

    # 컨트롤러 설정 파일
    controller_config = os.path.join(pkg_share,'robots','urdf', 'config', 'rb5_controller.yaml')

    # Gazebo 실행 (factory plugin 포함)
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # robot_state_publisher 노드
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description],
        output = 'screen'
    )

    # ros2_control_node 실행 (robot_description + config.yaml)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen'
    )

    # Gazebo에 로봇 스폰
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    # 컨트롤러 로딩
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rb5_joint_controller'],
        output='screen'
    )
    
    # RViz 설정 파일 (선택 사항)
    rviz_config_file = os.path.join(pkg_share, 'robots', 'urdf', 'rviz', 'rb5_rviz.rviz')

    # RViz 노드
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],  # 설정 파일 없으면 이 줄 생략 가능
        output='screen'
    )


    return LaunchDescription([
        gazebo,
        rsp_node,
        ros2_control_node,
        spawn_robot,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        rviz_node
    ])
