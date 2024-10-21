#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
import math
from turtlesim.srv import Spawn,Kill
from functools import partial
from my_robot_interfaces.msg import Turtle ,TurtleArray
from my_robot_interfaces.srv import CatchTurtle


class SpawnerNode(Node):
    def __init__(self):
        super().__init__("spawner_node")

        self.declare_parameter("spawn_speed" , 1.5)
        self.spawnSpeed_ = self.get_parameter("spawn_speed").value

        self.turtles_ = []
        self.turtleCounter_ = 1

        self.aliveTurtlePublisher_ = self.create_publisher(TurtleArray , "alive_turtles" , 10)
        self.spawnTimer_ = self.create_timer(self.spawnSpeed_ ,self.spawnNewTurtle)
        self.catchServer_ = self.create_service(CatchTurtle ,"catch_service" ,self.callbackCatchService)
        self.get_logger().info("spawner has been started")

    
    def callbackCatchService(self , request , response):
        self.callKillService(request.name)
        response.success = True
        return response

    def spawnNewTurtle(self):
        self.turtleCounter_ += 1
        name = "turtle" + str(self.turtleCounter_)
        x = random.uniform(0.0 , 11.0)
        y = random.uniform(0.0 , 11.0)
        theta = random.uniform(0.0 , 2*math.pi)
        self.callSpawnService(name, x, y, theta)


    def callSpawnService(self , name , x ,y ,theta):
        client = self.create_client(Spawn , "spawn")
        while not client.wait_for_service(2.0):
            self.get_logger().warn("waiting for server 'add two int'")

        request = Spawn.Request()
        request.name = name
        request.x = x
        request.y = y
        request.theta = theta

        future = client.call_async(request)
        future.add_done_callback(partial(self.callbackSpawnService ,name=name,x=x,y=y,theta=theta))

    def callbackSpawnService(self ,future , name , x , y , theta):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info(str(response.name) + " spawned successfully")
                turtle = Turtle()
                turtle.name = name
                turtle.x = x
                turtle.y = y
                turtle.theta = theta
                self.turtles_.append(turtle)
                self.publishAliveTurtles()
        except Exception as e:
            self.get_logger().error("service call field %r" % (e,))

    def callKillService(self , name):
        client = self.create_client(Kill , "kill")
        while not client.wait_for_service(2.0):
            self.get_logger().warn("waiting for server 'add two int'")


        request = Kill.Request()
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callbackKillService , name=name))

    def callbackKillService(self , future , name):
        try:
            future.result()
            self.get_logger().info(str(name) + " was killed successfully")
            for turtle in self.turtles_:
                if turtle.name == name:
                    self.turtles_.remove(turtle)
                    self.publishAliveTurtles()
                    break
        except Exception as e:
            self.get_logger().error("service call field %r" % (e,))

    def publishAliveTurtles(self):
        msg = TurtleArray()
        msg.arr = self.turtles_
        self.aliveTurtlePublisher_.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node = SpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()

