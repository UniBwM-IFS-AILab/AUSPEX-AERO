#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    # number of nodes to start
    # good reference here: https://answers.ros.org/question/376816/how-to-pass-launch-args-dynamically-during-launch-time/ <-- argv hack should work as well
    # and https://answers.ros.org/question/322874/ros2-what-is-different-between-declarelaunchargument-and-launchconfiguration/
    # and https://answers.ros.org/question/396345/ros2-launch-file-how-to-convert-launchargument-to-string/ <-- chives_onion answer
    
    count = int(context.perform_substitution(LaunchConfiguration('count')))
    
    launch_array =  []
 
    #print('The drone count is ' + str(count))

    # if arguments contains only the "" empty string, then it is completely left out and the next argument "--ros-args" replaces its position in argv[1]
    if count < 2:
        name_prefix = "vhcl0"
        launch_array.append(
            Node(
                package='auspex_obc',
                executable='auspex_obc',
                output='screen',
                shell=True,
                arguments=[name_prefix]
            )
        )
    else:
        for x in range(0, count):
            name_prefix = 'vhcl'+ str(x)
            launch_array.append(
                Node(
                    package='auspex_obc',
                    executable='auspex_obc',
                    output='screen',
                    shell=True,
                    arguments=[name_prefix]
                )
            )
    return launch_array

# declare command line argument via appending count:=NUMBER when starting the launch file from terminal
def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument('count', default_value='1')] + [OpaqueFunction(function=launch_setup)]
    )
