"""demo.launch.py — 통합 데모.

ros2 launch omx_reactor demo.launch.py \\
    camera:=v4l2|file|external|gazebo \\
    [file_path:=...] \\
    [robot:=sim|real] \\
    [port_name:=...]
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_arg = DeclareLaunchArgument(
        'camera', default_value='v4l2',
        description='v4l2|file|external|gazebo')

    robot_arg = DeclareLaunchArgument(
        'robot', default_value='sim',
        description='sim → Gazebo / real → omx_f follower')

    port_arg = DeclareLaunchArgument(
        'port_name',
        default_value=(
            '/dev/serial/by-id/'
            'usb-ROBOTIS_OpenRB-150_9F3EB2F35157375037202020FF112D0D-if00'
        ),
        description='real 모드 전용 — follower OpenRB-150 port')

    video_device_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0',
        description='camera=v4l2 일 때 사용할 V4L2 device (예: /dev/video2 = 노트북 HD Webcam)')

    # OMX_ROBOT 환경 변수 — trajectories.py import 시점에 분기
    omx_env = SetEnvironmentVariable(
        name='OMX_ROBOT', value=LaunchConfiguration('robot'))

    pkg_share = FindPackageShare('omx_reactor')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share, 'launch',
                ['camera_', LaunchConfiguration('camera'), '.launch.py'],
            ])
        ),
        launch_arguments={'video_device': LaunchConfiguration('video_device')}.items(),
    )

    omx_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'omx_gazebo.launch.py'])
        ),
        condition=UnlessCondition(
            PythonExpression(["'", LaunchConfiguration('robot'), "' == 'real'"])
        ),
    )

    omx_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'omx_real.launch.py'])
        ),
        launch_arguments={'port_name': LaunchConfiguration('port_name')}.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('robot'), "' == 'real'"])
        ),
    )

    geva = Node(package='dobi_npc_emotion', executable='geva_node',
                name='geva_node', output='screen',
                parameters=[{'input_topic': '/webcam/image_raw'}])

    rapport = Node(package='dobi_npc_emotion', executable='rapport_tracker',
                   name='rapport_tracker_node', output='screen',
                   parameters=[{'ema_alpha_base': 0.7, 'conf_min_gate': 0.2}])

    reactor = Node(package='omx_reactor', executable='reactor_node',
                   name='omx_reactor_node', output='screen')

    # dashboard MJPEG 입력 토픽: sim 은 /external_cam/image, real 은 /webcam/image_raw
    dashboard = Node(
        package='omx_reactor', executable='dashboard_node',
        name='omx_dashboard_node', output='screen',
        parameters=[{
            'http_port': 7700,
            'mjpeg_input_topic': PythonExpression([
                "'/webcam/image_raw' if '",
                LaunchConfiguration('robot'),
                "' == 'real' else '/external_cam/image'",
            ]),
        }])

    gesture = Node(package='omx_reactor', executable='gesture_detector_node',
                   name='gesture_detector_node', output='screen',
                   parameters=[{
                       'cooldown_sec': 5.0,
                       'up_threshold': 0.55,
                       'wave_window_size': 10,
                       'wave_oscillation_threshold': 0.15,
                   }])

    return LaunchDescription([
        camera_arg,
        robot_arg,
        port_arg,
        video_device_arg,
        omx_env,
        camera_launch,
        omx_sim,
        omx_real,
        geva,
        rapport,
        reactor,
        dashboard,
        gesture,
    ])
