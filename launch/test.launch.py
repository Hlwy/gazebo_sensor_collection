import os
import sys
import time
import yaml
import xacro
import pathlib
from platform import python_version

from loguru import logger
logger.remove()
logger.add(sys.stdout, level="DEBUG", colorize=True, format="<level>[{level: <8}] {time:h:mm:ss.SSS} - {message}</level>")

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix, get_package_share_path

from launch import LaunchDescription
from launch.action import Action
from launch.actions import OpaqueFunction
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import Command, PythonExpression
from launch.substitutions import FindExecutable, LaunchConfiguration

from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource

#Helper function to read a yaml config file
def load_yaml_file(yaml_file_path):
    try:
        with open(yaml_file_path, 'r') as file:
            return yaml.load(file)
    except EnvironmentError as e: # parent of IOError, OSError *and* WindowsError where available
        print(str(e))
        return None

def launch_setup(context, *args, **kwargs):
    # #Example: Read the config yaml file and do something based on the values
    # joy_config = PathJoinSubstitution(
    #     [FindPackageShare('gazebo_sensor_collection'), "config", "joy-params.yaml"]
    # )
    # #Read the yaml file's ros parameters
    # joy_params = load_yaml_file(str(joy_config.perform(context)))['joy']['ros__parameters']
    # #You an build login based on the config file values to adapt your launch file behavior
    # print("Joy Params: ")
    # print(joy_params)

    gui = LaunchConfiguration('gui')
    server = LaunchConfiguration('server')
    use_sim_time = LaunchConfiguration('use_sim_time')
    pause = LaunchConfiguration('pause')
    verbose = LaunchConfiguration('verbose')

    #Example: Get the launch file arguments declared in the generate_launch_description() method
    prefix = LaunchConfiguration('prefix')
    model_file = LaunchConfiguration('model_file')
    description_package = LaunchConfiguration('description_package')

    gdb = LaunchConfiguration('gdb')
    world = LaunchConfiguration('world')
    world_file = LaunchConfiguration('world_file')
    world_package = LaunchConfiguration('world_package')

    _description_pkg = str(LaunchConfiguration('description_package').perform(context))
    _model_file = str(LaunchConfiguration('model_file').perform(context))
    _world = str(LaunchConfiguration('world').perform(context))
    _world_file = str(LaunchConfiguration('world_file').perform(context))
    _world_package = str(LaunchConfiguration('world_package').perform(context))

    my_pkg_install_dir = get_package_prefix('gazebo_sensor_collection')

    m_world_pkg = None
    defaultWorldPkg = pathlib.Path("/usr/share/gazebo-11")
    if(_world_package == ""):
        # print(f" arg-in \'world\' is empty, ovrriding launch arg world:={defaultWorldPkg}")
        m_world_pkg = str(defaultWorldPkg)
    else:
        # print(f" arg-in \'world\' is non-empty, using provided arg world:={_world_package}")
        m_world_pkg = world_package

    world_path = PathJoinSubstitution(
        [m_world_pkg, "worlds", world_file]
    ).perform(context)
    _world_path = str(world_path)

    m_world = None
    if(_world == ""):
        # print(f" arg-in \'world\' is empty, ovrriding launch arg world:={_world_path}")
        m_world = world_path
    else:
        # print(f" arg-in \'world\' is non-empty, using provided arg world:={_world}")
        m_world = world

    # if('GAZEBO_MODEL_PATH' in os.environ):
    #     os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + my_pkg_install_dir + '/share'
    # else: os.environ['GAZEBO_MODEL_PATH'] =  my_pkg_install_dir + "/share"
    #
    # if 'GAZEBO_PLUGIN_PATH' in os.environ: os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + my_pkg_install_dir + '/lib'
    # else: os.environ['GAZEBO_PLUGIN_PATH'] = my_pkg_install_dir + '/lib'

    print(f"Launch Args from context:")
    print(f" -- description_pkg   = {_description_pkg}")
    print(f" -- model_file        = {_model_file}")
    print(f" -- world             = {_world}")
    print(f" -- world_file        = {_world_file}")
    print(f" -- world_package     = {_world_package}")
    print(f" -- world_path        = {_world_path}")
    # print(f" -- worlds_pkg_path = {worlds_pkg_path}")
    # print(f"")
    print(f" ----------- ")

    #Example: Access the value of a launch argument from the LaunchContext
    #Example: Use conditionals
    #Usage: ros2 launch ros2_launch_tutorial ros2.launch.py conditional_demo:=false
    if(IfCondition(LaunchConfiguration('conditional_demo')).evaluate(context)): print("The conditional_demo value was set to True")
    else: print("The conditional_demo value was set to False")

    #Example: Get URDF via xacro
    model_path = None
    _model_path = ""
    model_paths = [
        PathJoinSubstitution([FindPackageShare(description_package), "launch/include", model_file]),
        PathJoinSubstitution([FindPackageShare(description_package), "robots", model_file]),
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", model_file]),
    ]
    _model_paths = []
    for mp in model_paths:
        tpath = pathlib.Path(str(mp.perform(context)))
        if(not tpath.exists()):
            # print(f" - discarding non-existant model at path = {str(tpath)}")
            continue
        else: print(f" - found model_path = {str(tpath)}")
        _model_paths.append(tpath)
    if(len(_model_paths) > 0):
        model_path = _model_paths[0]
        _model_path = str(model_path)
    print(f" -- using model at path = {_model_path}")

    # robot_description_content = Command([
    #     PathJoinSubstitution([FindExecutable(name="xacro")]), " ", str(model_path), " ", "prefix:=", prefix,
    # ])
    # robot_description_param = {'robot_description': robot_description_content}
    # robot_description_content = xacro.process_file(_model_path )#, mappings={'prefix': prefix.perform(context)} )
    robot_description_content = xacro.process(
        _model_path,
        mappings={'prefix': prefix.perform(context)},
        xacro_ns=False
    )
    robot_description_param = {
        # 'robot_description': robot_description_content.toxml()
        'robot_description': robot_description_content
    }
    robot_description_param_act = SetParameter(name='robot_description', value=robot_description_content)

    pkg_share = FindPackageShare(package='gazebo_sensor_collection').find('gazebo_sensor_collection')
    # Specify the actions
    # Start Gazebo server w/ some of the avaialble key arg inputs:
    # -- verbose,
    # -- lockstep, pause,
    # -- physics - physics engines (ode|bullet|dart|simbody)
    # -- init, factory, force_system
    # -- profile, extra_gazebo_args
    # -- gdb, valgrind
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource( os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py'), )
    # )

    gzserver_args = {'world': m_world, 'gdb': gdb, 'pause': pause, 'verbose': verbose }
    start_gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py') ),
        condition=IfCondition(server), launch_arguments=gzserver_args.items()
    )
    # Start Gazebo client   
    gzclient_args = {'verbose': verbose}
    start_gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py') ),
        condition=IfCondition(gui), launch_arguments=gzclient_args.items()
    )

    swpan_args = '{name: \"robot_description\", xml: \"'  +  robot_description_content.replace('"', '\\"') + '\" }'
    spawn_entity_cmd = ExecuteProcess(output='screen',
        cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', swpan_args],
    )
    # robot_description_param_act = ExecuteProcess(output='screen',
    #     cmd=['ros2', 'param', 'set', '/spawn_entity', 'robot_description', robot_description_content.replace('"', '\\"')],
    # )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
        # arguments=['-entity', 'mulecar', '-file', 'descriptions/sdf/model.sdf'],
        arguments=["-topic", "robot_description", "-entity", "robot", "-x", "0.0", "-y", "0.0", "-z", "0.0"],
        output='screen'
    )
    robot_state_publisher_node = Node(package='robot_state_publisher',
        executable='robot_state_publisher', name='robot_state_publisher',
        # parameters=[{'robot_description': robot_description_content}],
        parameters=[robot_description_param],
        output='both',
    )

    joint_state_publisher_node = Node(package='joint_state_publisher', executable='joint_state_publisher',
        name='joint_state_publisher', output='screen',
        # condition=UnlessCondition(use_gui)
    )
    # joint_state_publisher_node = Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
    #   name='joint_state_publisher', output='screen',
    #   condition=IfCondition(use_gui)
    # )


    # rviz_config = PathJoinSubstitution( [FindPackageShare('ros2_launch_tutorial'), "rviz", "rrbot.rviz"] )
    rviz_node = Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
        # arguments=["-d", rviz_config] # You can directly use the PathJoinSubstitution object
        arguments=[], condition=IfCondition(LaunchConfiguration('rviz'))
    )
    rqt_node = Node(package='rqt_gui', executable='rqt_gui',
        # arguments=['-d', os.path.join(pkg_custom_pkg, 'rviz', 'dolly_gazebo.rviz')],
        arguments=[], condition=IfCondition(LaunchConfiguration('rqt'))
    )
    joy_node = Node(package='joy', executable='joy_node', name='joy', output='both',
        # parameters=[joy_config]
    )
    return [
        # gazebo,
        start_gzserver, start_gzclient,
        # robot_description_param_act,
        spawn_entity,
        # spawn_entity_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node, rqt_node
    ]

# Main function serving as a starting point for the launch file
def generate_launch_description():
    return LaunchDescription([
        #Declare all arguments here. The arguments can be accessed using LaunchConfiguration() in the launch_setup() method
        DeclareLaunchArgument("description_package",
            default_value="gazebo_sensor_collection",
        ),
        DeclareLaunchArgument("model_file",
            default_value="sensors_attachments.urdf.xacro",
        ),
        DeclareLaunchArgument("prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
            multi-robot setup. If changed than also joint names in the controllers' configuration \
            have to be updated."
        ),
        DeclareLaunchArgument("conditional_demo",
            default_value="true",
            description="A demo argument to show an example of the IfCondition module."
        ),

        DeclareLaunchArgument('gdb', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('server', default_value='true'),
        DeclareLaunchArgument('pause', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument("world", default_value=""),
        DeclareLaunchArgument("world_package", default_value=""),
        DeclareLaunchArgument("world_file", default_value="cafe.world"),
        DeclareLaunchArgument('rqt', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),

        OpaqueFunction(function = launch_setup)
        ])
