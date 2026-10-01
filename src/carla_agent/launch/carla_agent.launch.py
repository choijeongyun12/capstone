from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host_arg       = DeclareLaunchArgument('host',       default_value='127.0.0.1')
    port_arg       = DeclareLaunchArgument('port',       default_value='2000')
    map_arg        = DeclareLaunchArgument('map',        default_value='Town04_Opt')
    target_kph_arg = DeclareLaunchArgument('target_kph', default_value='45.0')

    host       = LaunchConfiguration('host')
    port       = LaunchConfiguration('port')
    map_       = LaunchConfiguration('map')
    target_kph = LaunchConfiguration('target_kph')

    spawner = Node(
        package='carla_agent',
        executable='vehicle_spawner',
        name='vehicle_spawner',
        output='screen',
        parameters=[{
            'host':           host,
            'port':           port,
            'map':            map_,
            'vehicle_filter': 'vehicle.tesla.model3',
            'role_name':      'ego_vehicle',
            'spawn_index':    0,
            'autopilot':      False,   # 고전 비전 제어 사용
            'cam_width':      640,
            'cam_height':     480,
            'cam_fov':        90.0,
            'radar_range':    50.0,
            'radar_hfov':     30.0,
            'radar_vfov':     10.0,
        }],
    )

    # 차량 스폰 완료 후 5초 뒤 차선추종 노드 시작
    lane_follower = TimerAction(
        period=5.0,
        actions=[Node(
            package='carla_agent',
            executable='lane_follower',
            name='lane_follower',
            output='screen',
            parameters=[{
                'host':          host,
                'port':          port,
                'role_name':     'ego_vehicle',
                'target_kph':    target_kph,
                'steer_gain':    1.2,
                'radar_slow_m':  15.0,
                'radar_brake_m':  8.0,
            }],
        )],
    )

    visualizer = TimerAction(
        period=5.0,
        actions=[Node(
            package='carla_agent',
            executable='visualizer',
            name='visualizer',
            output='screen',
            parameters=[{
                'panel_width':  640,
                'panel_height': 480,
            }],
        )],
    )

    spectator = TimerAction(
        period=5.0,
        actions=[Node(
            package='carla_agent',
            executable='spectator_follower',
            name='spectator_follower',
            output='screen',
            parameters=[{
                'host':       host,
                'port':       port,
                'role_name':  'ego_vehicle',
                'offset_x':   -8.0,   # 뒤쪽 거리 (m)
                'offset_z':    5.0,   # 높이 (m)
                'pitch':      -15.0,  # 내려다보는 각도
                'alpha_pos':   0.15,
                'alpha_rot':   0.10,
            }],
        )],
    )

    object_detector = TimerAction(
        period=15.0,   # UFLD 로딩 완료 후 시작 (GPU 경합 방지)
        actions=[Node(
            package='carla_agent',
            executable='object_detector',
            name='object_detector',
            output='screen',
        )],
    )

    return LaunchDescription([
        host_arg,
        port_arg,
        map_arg,
        target_kph_arg,
        spawner,
        lane_follower,
        object_detector,
        visualizer,
        spectator,
    ])
