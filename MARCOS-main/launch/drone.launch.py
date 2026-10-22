import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

DRONES = [
    {"ns": "drone0", "id": 0, "pose": "0,0,0,0,0,0",  "scan_topic": "scan",   "depth_topic": "depth_camera"},
    {"ns": "drone1", "id": 1, "pose": "-3,2,0,0,0,0",  "scan_topic": "scan_1", "depth_topic": "depth_camera_1"},
    {"ns": "drone2", "id": 2, "pose": "-3,-2,0,0,0,0", "scan_topic": "scan_2", "depth_topic": "depth_camera_2"},
]

def gz_model_name(d):
    return 'x500_depth' if d['id'] == 0 else f"x500_depth_{d['id']}"

def generate_launch_description():
    package_name = 'autonomous_drone'
    pkg_share = get_package_share_directory(package_name)
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'mapping.rviz')
    home_dir = os.environ.get('HOME')
    px4_dir = os.path.join(home_dir, 'px4_ros2_ws', 'PX4-Autopilot')
    px4_binary = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')
    GZ_MODELS_PATH = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
    GZ_WORLDS_PATH = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'worlds')
    sim_time_config = {'use_sim_time': True}

    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], output='screen')

    # THE one and only /clock bridge (latched)
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{
            'qos_overrides./clock.publisher.reliability': 'reliable',
            'qos_overrides./clock.publisher.durability': 'transient_local',
            'qos_overrides./clock.publisher.history': 'keep_last',
            'qos_overrides./clock.publisher.depth': 10,
        }],
        output='screen')

    # PX4 instances — staggered
    px4_timers = []
    for idx, d in enumerate(DRONES):
        env = os.environ.copy()
        env['PX4_SYS_AUTOSTART'] = '4002'
        env['PX4_GZ_MODEL'] = gz_model_name(d)
        env['PX4_GZ_MODEL_POSE'] = d['pose']
        env['PX4_UXRCE_DDS_NS'] = d['ns']
        env['GZ_SIM_RESOURCE_PATH'] = (GZ_MODELS_PATH + ':' + GZ_WORLDS_PATH + ':'
                                       + env.get('GZ_SIM_RESOURCE_PATH', ''))
        if d['id'] != 0:
            env['PX4_GZ_STANDALONE'] = '1'
        px4_timers.append(TimerAction(
            period=5.0 + 10.0 * idx,
            actions=[ExecuteProcess(cmd=[px4_binary, '-i', str(d['id'])],
                                    cwd=px4_dir, env=env, output='screen')]))

    # Per-drone bridges (NO clock here)
    bridges = []
    for d in DRONES:
        bridges.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name=f"ros_gz_bridge_{d['ns']}", namespace=d['ns'],
            arguments=[
                f"/{d['depth_topic']}@sensor_msgs/msg/Image@gz.msgs.Image",
                f"/{d['scan_topic']}@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            ],
            remappings=[
                (f"/{d['depth_topic']}", 'depth_camera'),
                (f"/{d['scan_topic']}", 'scan'),
            ],
            output='screen'))
    bridges_timer = TimerAction(period=35.0, actions=bridges)

    # Per-drone autonomy groups
    drone_groups = []
    for d in DRONES:
        x, y, z, _, _, yaw = [float(v) for v in d['pose'].split(',')]
        drone_groups.append(GroupAction([
            PushRosNamespace(d['ns']),
            Node(package=package_name, executable='depth_avoidance', name='depth_avoidance',
                 parameters=[sim_time_config], output='screen'),
            Node(package=package_name, executable='mapping_node', name='mapping_node',
                 parameters=[sim_time_config], output='screen'),
            Node(package=package_name, executable='path_planner', name='path_planner',
                 parameters=[sim_time_config],
                 remappings=[('map', '/map')],        # planner plans on COLLECTIVE map
                 output='screen'),
            Node(package=package_name, executable='offboard_control', name='offboard_control',
                 parameters=[sim_time_config], output='screen'),
            # >>> THIS replaces the old static_transform_publisher <<<
            Node(package=package_name, executable='odom_tf', name='odom_tf',
                 parameters=[sim_time_config,
                             {'base_frame': f"{d['ns']}/base_link",
                              'lidar_frame': f"x500_depth_{d['id']}/lidar_link_custom/lidar_custom",
                              'spawn_x': x, 'spawn_y': y, 'spawn_z': z, 'spawn_yaw': yaw}],
                 remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
                 output='screen'),
        ]))

    # >>> THE COLLECTIVE MAP BUILDER — top-level, no namespace <<<
    map_merger = Node(package=package_name, executable='map_merger', name='map_merger',
                      parameters=[sim_time_config], output='screen')

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[sim_time_config], output='screen')
    rviz_timer = TimerAction(period=45.0, actions=[rviz_node])

    autonomy_timer = TimerAction(period=40.0, actions=[map_merger, *drone_groups])

    return LaunchDescription([
        micro_xrce_agent,
        clock_bridge,
        *px4_timers,
        bridges_timer,
        autonomy_timer,
        rviz_timer,
    ])