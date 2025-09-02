from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    eater_turtle_ns = "nine"
    killer_turtle_ns = "ioon"
    sampling_frequency = 100.0

    turtlesim_plus_node = Node(
            package='turtlesim_plus',
            executable='turtlesim_plus_node.py',
            name='turtlesim_plus',
            output='screen'
        )
    
    eater_node = Node(
            package='lab3',
            executable='eater.py',
            name='eater_node',
            namespace=eater_turtle_ns,
            parameters=[{
                "sampling_frequency": sampling_frequency
            }],
            output='screen'
        )
    
    killer_node = Node(
            package='lab3',
            executable='killer.py',
            name='killer_node',
            namespace=killer_turtle_ns,
            parameters=[{
                "sampling_frequency": sampling_frequency,
                "kill_turtle": eater_turtle_ns
            }],
            output='screen'
        )
    
    kill_turtle1 = ExecuteProcess(cmd=[['ros2 service call /remove_turtle turtlesim/srv/Kill ' + "'name: 'turtle1''"]],
                                  shell=True)
    spawn_eater = ExecuteProcess(cmd=[['ros2 service call /spawn_turtle turtlesim/srv/Spawn ' + f"'name: '{eater_turtle_ns}''"]],
                                 shell=True)
    spawn_killer = ExecuteProcess(cmd=[['ros2 service call /spawn_turtle turtlesim/srv/Spawn ' + f"'name: '{killer_turtle_ns}''"]],
                                  shell=True)
    
    ld = LaunchDescription()
    ld.add_action(turtlesim_plus_node)

    ld.add_action(kill_turtle1)
    ld.add_action(spawn_eater)
    ld.add_action(spawn_killer)

    ld.add_action(eater_node)
    ld.add_action(killer_node)

    return ld