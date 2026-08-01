import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    pkg_rover_description = get_package_share_directory('rover_description')
    
    gz_resource_path = SetEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value=os.path.join(pkg_rover_description, '..')
    )

    xacro_file = os.path.join(pkg_rover_description, 'urdf', 'rover.urdf.xacro')
    world_path = os.path.join(pkg_rover_description, 'worlds', 'room.sdf')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Publishes robot_description + TF tree, same as your RViz setup
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Launch Gazebo (gz sim) with an empty world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # Spawn the robot into the running Gazebo world using robot_description
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'rover'],
        output='screen'
    )

    # Bridge cmd_vel and odom between ROS 2 and Gazebo transport
    #bridge = Node(
     #   package='ros_gz_bridge',
      #  executable='parameter_bridge',
       # arguments=[
        #    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
         #   '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        #],
       # output='screen'
    #)

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen'
                )
            ]
        )
    )
    
    load_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_drive_controller'],
                    output='screen'
                )
            ]
        )
    )
    
    lidar_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'rover/base_footprint/lidar'],
        output='screen'
    )
    
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        output='screen'
    )
    
    ground_truth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/rover/odometry_ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        output='screen'
    )
    
    return LaunchDescription([
        gz_resource_path,
        robot_state_publisher_node,
        gz_sim,
        spawn_entity,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        clock_bridge,
        lidar_frame_bridge,
        lidar_bridge,
        imu_bridge,
        ground_truth_bridge
    ])