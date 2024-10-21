#!/usr/bin/env python3
from doctest import Example

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.srv import CatchTurtle
from my_robot_interfaces.msg import Turtle , TurtleArray

from functools import partial
import math
class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.declare_parameter("catch_the_nearest_turtle" , True)
        self.isCatchingNearest_ = self.get_parameter("catch_the_nearest_turtle").value

        self.target_ = None
        self.position_ = None

        self.publisher_ = self.create_publisher(Twist , "turtle1/cmd_vel" , 10)
        self.timer_ = self.create_timer(0.01 , self.publishMoveCommand)
        self.poseSubscriber_ = self.create_subscription(Pose , "turtle1/pose" , self.callbackPose ,10)
        self.aliveTurtleSubscriber_ = self.create_subscription(TurtleArray, "alive_turtles", self.callbackAliveTurtles, 10)
        self.get_logger().info("py spawner node has been started")

    def callbackAliveTurtles(self, msg):
        if len(msg.arr) > 0:
            if self.isCatchingNearest_:
                minDistance = None
                minTurtleDistance = None

                for turtle in msg.arr:
                    distX = turtle.x - self.position_.x
                    distY = turtle.y - self.position_.y
                    distance = math.sqrt(distX*distX + distY*distY)
                    if minDistance is None or distance < minDistance:
                        minDistance = distance
                        minTurtleDistance = turtle
                self.target_ = minTurtleDistance
            else:
                self.target_ = msg.arr[0]
    def callbackPose(self , msg):
        self.position_ = msg
    def publishMoveCommand(self):
        if self.target_ is None or self.position_ is None:
            return
        try:
            distX = self.target_.x - self.position_.x
            distY = self.target_.y - self.position_.y
            distance = math.sqrt(distX * distX + distY * distY)

            msg = Twist()

            if distance > 0.5:
                msg.linear.x = 2*distance

                targetTheta = math.atan2(distY , distX)
                diff = targetTheta - self.position_.theta

                if diff > math.pi:
                    diff -= 2*math.pi
                elif diff < -math.pi:
                    diff += 2*math.pi
                msg.angular.z = 6*diff
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.callCatchService(self.target_.name)
                self.target_ = None

            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error("%r" % (e,))

    def callCatchService(self , turtleName):
        client = self.create_client(CatchTurtle ,"catch_service")
        while not client.wait_for_service(2.0):
            self.get_logger().warn("waiting for server 'add two int'")

        request = CatchTurtle.Request()
        request.name = turtleName
        future = client.call_async(request)
        future.add_done_callback(partial(self.callbackCatchService , turtleName = turtleName))

    def callbackCatchService(self , future , turtleName):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(str(turtleName) + " cannot be caught")
        except Example as e:
            self.get_logger().error("service call field %r" % (e,))


def main(args = None):
    rclpy.init(args = args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()

