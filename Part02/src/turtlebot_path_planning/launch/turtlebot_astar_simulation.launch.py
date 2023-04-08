from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from math import pi
import os


#This function decrustrates the start_node launch argument and passes it to the spawn_turtlebot3.launch.py file:
def turtlebot_spawn(context: LaunchContext, start_node):
    arr = context.perform_substitution(start_node)
    x = arr.replace('[', "").replace(']',"").split(',')[0] #Remove the brackets and split the string
    y = arr.replace('[', "").replace(']',"").split(',')[1]
    angle = arr.replace('[', "").replace(']',"").split(',')[2]
    rad = (90.0-float(angle)) * pi / 180 #Subtract 90 degrees from the anlge for proper spawning & Convert angle to radians
    
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x,
            'y_pose': y,
            'theta': str(rad)
        }.items()
    )

    return [spawn_turtlebot_cmd] #Return the spawn_turtlebot_cmd to be added to the launch description



#This function generates the launch description - basic structure of the launch file
def generate_launch_description(): 

    #Declare the launch arguments:
    start_node = DeclareLaunchArgument(
        'start_node',
        default_value='[0.0, 0.0, 0.0]',
        description='start node for turtlebot'
    )

    goal_node = DeclareLaunchArgument(
        'goal_node',
        default_value='[0.0, 0.0, 0.0]',
        description='goal node for turtlebot'
    )

    RPM1 = DeclareLaunchArgument(
        'RPM1',
        default_value='0.0',
        description='RPM1 for turtlebot'
    )

    RPM2 = DeclareLaunchArgument(
        'RPM2',
        default_value='0.0',
        description='RPM2 for turtlebot'
    )

    clearance = DeclareLaunchArgument(
        'clearance',
        default_value='0.0',
        description='clearance for turtlebot'
    )

    
    #Declare nodes - vel_pub node which performs the path planning & publishes the velocity to the turtlebot
    #It accepts the start_node, goal_node, RPM1, RPM2, and clearance as launch arguments
    velocity_publisher = Node(
        package='velocity_publisher',
        executable='vel_pub',
        parameters=[
            {'start_node': LaunchConfiguration('start_node')},
            {'goal_node': LaunchConfiguration('goal_node')},
            {'RPM1': LaunchConfiguration('RPM1')},
            {'RPM2': LaunchConfiguration('RPM2')},
            {'clearance': LaunchConfiguration('clearance')}
        ],
        output='screen'
    )
    

    #Create the launch description
    ld = LaunchDescription()
    
    #Declare launch files to be launch - within this launch file
    #Gazebo Turtlebot3 World is launched with the map.world file
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'map.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )


    #Add the launch actions to the launch description - do it for every launch action
    ld.add_action(gzserver_cmd) 
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)

    ld.add_action(OpaqueFunction(function=turtlebot_spawn, args=[LaunchConfiguration('start_node')])) #Add the spawn_turtlebot_cmd to the launch description

    ld.add_action(start_node)
    ld.add_action(goal_node)
    ld.add_action(RPM1)
    ld.add_action(RPM2)
    ld.add_action(clearance)

    ld.add_action(velocity_publisher) #Add the velocity_publisher package
    
    return ld #Return the launch description
