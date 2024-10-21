from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    numberPublisherNode = Node(
        package = "my_py_pkg",
        executable ="number_publisher",
        name="my_number_publisher" ,#for remapping the node name.
        remappings = [#for remapping topics , service etc
            ("number" , "my_number")
        ] ,
        parameters = [#for passing argument.
            {"number_to_publish" : 4}
        ]
    )
    numberCounterNode = Node(
        package="my_cpp_pkg",
        executable="number_counter",
        name = "my_number_counter" ,
        remappings = [
            ("number" , "my_number"),
            ("number_count" , "my_number_count")
        ]
    )

    ld.add_action(numberPublisherNode)
    ld.add_action(numberCounterNode)
    return ld