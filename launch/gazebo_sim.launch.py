import os
import sys
import time
import xacro
import pathlib
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    gz_share_dir = ""
    for p in pathlib.Path("/usr/share").glob('*gazebo*/world*'):
        gz_share_dir = str(p.parent)
    if(gz_share_dir == ""): print("failed to find gazebo root share dir..")
    else: print(f"found gazebo worlds dir = {gz_share_dir}")

    # Set the path to the world file
    world_file_name = 'cafe.world'
    world_path = os.path.join(gz_share_dir, 'worlds', world_file_name)

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_custom_pkg = get_package_share_directory('gazebo_sensor_collection')
    install_dir = get_package_prefix('gazebo_sensor_collection')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model_name = 'sensors_attachments.urdf.xacro'

    # pkg_box_car_description = get_package_share_directory('box_car_description')
    print(pkg_custom_pkg)
    xacro_file = os.path.join(str(pkg_custom_pkg), 'launch/include/', str(model_name))
    # print(xacro_file)
    assert os.path.exists(xacro_file), "The box_bot.xacro doesnt exist in "+str(xacro_file)


    if 'GAZEBO_MODEL_PATH' in os.environ: os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else: os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ: os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else: os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()
    # print(robot_desc)
    robot_desc_param = {'robot_description': robot_description_config.toxml()}

    # # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # Follow node
    # follow = Node(
    #     package='dolly_follow',
    #     executable='dolly_follow',
    #     output='screen',
    #     remappings=[
    #         ('cmd_vel', '/dolly/cmd_vel'),
    #         ('laser_scan', '/dolly/laser_scan')
    #     ]
    # )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[],
        # arguments=['-d', os.path.join(pkg_custom_pkg, 'rviz', 'dolly_gazebo.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    rqt = Node(
        package='rqt_gui',
        executable='rqt_gui',
        arguments=[],
        # arguments=['-d', os.path.join(pkg_custom_pkg, 'rviz', 'dolly_gazebo.rviz')],
        condition=IfCondition(LaunchConfiguration('rqt'))
    )

    spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
                # arguments=['-entity', 'mulecar', '-file', 'descriptions/sdf/model.sdf'],
                arguments=["-topic", "robot_description", "-entity", "robot", "-x", "0.0", "-y", "0.0", "-z", "0.0"],
                output='screen')
    bot_state_pubber = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_desc_param],
        output="screen")

    time.sleep(5.0)
    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_gazebo_ros, 'worlds', 'empty_world.world'), ''],
          description='SDF world file'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        DeclareLaunchArgument('rqt', default_value='false', description='Open Rqt.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo,
        spawn_entity,
        bot_state_pubber,
        #start_steering_control,
        # follow,
        rviz, rqt
    ])