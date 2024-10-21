from sys import executable

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    turtleSimNode = Node(
        package= "turtlesim",
        executable="turtlesim_node"
    )
    controllerNode = Node(
        package="cpp_final_pro_udemy",
        executable="controller",
        parameters=[
            {"catch_the_nearest_turtle": True}
        ]
    )
    spawnerNode = Node(
        package="cpp_final_pro_udemy",
        executable="spawner" ,
        parameters =[
            {"spawn_speed" : 1}
        ]
    )

    ld.add_action(turtleSimNode)
    ld.add_action(controllerNode)
    ld.add_action(spawnerNode)
    return ld