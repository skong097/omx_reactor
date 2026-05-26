"""omx_real — 실 OMX (omx_f follower) bringup (race-free 직접 spawn).

ROBOTIS open_manipulator_bringup/omx_f.launch.py 가 ros2_control_node 와
robot_state_publisher 를 동시 띄워 race condition 발생 (controller_manager 가
robot_description topic 무한 대기). 우회: robot_state_publisher 먼저 띄우고
3s delay 후 ros2_control_node + spawner + init_position 시작.

NOTE: leader OpenRB-150 은 띄우지 않음 (kinesthetic teleop 미사용).
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DEFAULT_PORT = (
    '/dev/serial/by-id/'
    'usb-ROBOTIS_OpenRB-150_9F3EB2F35157375037202020FF112D0D-if00'
)


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port_name',
        default_value=DEFAULT_PORT,
        description='follower OpenRB-150 serial port (by-id symlink 권장)')

    init_position_arg = DeclareLaunchArgument(
        'init_position',
        default_value='true',
        description='omx_f init_position node (안전 자세부터 시작)')

    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='""',
        description='Joint / link name prefix (default 빈 문자열)')

    port_name = LaunchConfiguration('port_name')
    init_position = LaunchConfiguration('init_position')
    prefix = LaunchConfiguration('prefix')

    # URDF: xacro process — omx_f 5축 follower, use_sim=false
    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('open_manipulator_description'),
            'urdf', 'omx_f', 'omx_f.urdf.xacro',
        ]),
        ' ',
        'prefix:=', prefix, ' ',
        'use_sim:=false', ' ',
        'use_mock_hardware:=false', ' ',
        'mock_sensor_commands:=false', ' ',
        'port_name:=', port_name, ' ',
        'ros2_control_type:=omx_f',
    ])

    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config', 'omx_f', 'hardware_controller_manager.yaml',
    ])

    init_position_params = PathJoinSubstitution([
        FindPackageShare('open_manipulator_bringup'),
        'config', 'omx_f', 'initial_positions.yaml',
    ])

    # 1) robot_state_publisher 먼저 — robot_description topic transient_local publish
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_file, 'use_sim_time': False}],
        output='screen',
    )

    # 2) 3초 후 ros2_control_node + spawner — robot_state_publisher 가 안정화된 후
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': urdf_file}, controller_manager_config],
        output='screen',
    )

    spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            'gripper_controller',
            'joint_state_broadcaster',
        ],
        output='screen',
    )

    # 3) init_position — spawner 완료 후 trajectory 한 번 보냄
    joint_trajectory_executor = Node(
        package='open_manipulator_bringup',
        executable='joint_trajectory_executor',
        parameters=[init_position_params],
        output='screen',
        condition=IfCondition(init_position),
    )

    delayed_control = TimerAction(
        period=3.0,
        actions=[control_node, spawner],
    )

    # spawner 종료 후 init_position 실행
    init_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner,
            on_exit=[joint_trajectory_executor],
        )
    )

    return LaunchDescription([
        port_arg,
        init_position_arg,
        prefix_arg,
        robot_state_publisher_node,    # 즉시 시작
        delayed_control,                # 3s 후
        init_after_spawn,
    ])
