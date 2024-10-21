from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    topicName = ("robot_news" , "activity_6_topic_name")
    robotNews1Node = Node(
        package= "my_py_pkg",
        executable= "robot_news_station",
        name="robot_news_station_giskard",
        parameters = [
            {"robot_name" : "giskard"}
        ],
        remappings = [
            topicName
        ]
    )
    robotNews2Node = Node(
        package= "my_py_pkg",
        executable= "robot_news_station",
        name="robot_news_station_bb8",
        parameters = [
            {"robot_name" : "bb8"}
        ],
        remappings = [
            topicName
        ]
    )
    robotNews3Node = Node(
        package= "my_py_pkg",
        executable= "robot_news_station",
        name="robot_news_station_daneel",
        parameters = [
            {"robot_name" : "daneel"}
        ],
        remappings = [
            topicName
        ]
    )
    robotNews4Node = Node(
        package= "my_cpp_pkg",
        executable= "robot_news_station",
        name="robot_news_station_lander",
        parameters = [
            {"robot_name" : "lander"}
        ],
        remappings = [
            topicName
        ]
    )
    robotNews5Node = Node(
        package= "my_cpp_pkg",
        executable= "robot_news_station",
        name="robot_news_station_c3po",
        parameters = [
            {"robot_name" : "c3po"}
        ],
        remappings = [
            topicName
        ]
    )
    smartphoneNode = Node(
        package= "my_py_pkg" ,
        executable= "smartphone",
        name="smartphone",
        remappings = [
            topicName
        ]
    )
    ld.add_action(robotNews1Node) ,ld.add_action(robotNews2Node)
    ld.add_action(robotNews3Node), ld.add_action(robotNews4Node)
    ld.add_action(robotNews5Node) , ld.add_action(smartphoneNode)
    return ld