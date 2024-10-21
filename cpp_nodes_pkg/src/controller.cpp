
#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include <cmath>

using turtlesim::msg::Pose;
using my_robot_interfaces::msg::Turtle;
using my_robot_interfaces::msg::TurtleArray;
using my_robot_interfaces::srv::CatchTurtle;
using geometry_msgs::msg::Twist;
using std::placeholders::_1;
using std::placeholders::_2;


class SpawnerNode : public rclcpp::Node{
public:
    SpawnerNode(): Node("controller_node") , isControllerUp_(false) , isCatchingNearest_(true){
        this->declare_parameter("catch_the_nearest_turtle" , true);
        isCatchingNearest_ = this->get_parameter("catch_the_nearest_turtle").as_bool();
        
        publisher_ = this->create_publisher<Twist>("turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::microseconds(10),std::bind(&SpawnerNode::publishMoveCommand, this));
        poseSubscriber_ = this->create_subscription<Pose>("turtle1/pose", 10, std::bind(&SpawnerNode::callbackPose, this, _1));
        aliveTurtleSubscriber_ = this->create_subscription<TurtleArray>("alive_turtles", 10, std::bind(&SpawnerNode::callbackAliveTurtles, this, _1));
        RCLCPP_INFO(this->get_logger() , "controller node has been started");
        
    }
    
private:
    double getDistFromPose(const Turtle turtle){
        double distX = turtle.x - position_.x;
        double distY = turtle.y - position_.y;
        return std::sqrt(distX*distX + distY*distY);
    } 
    void callbackAliveTurtles(const TurtleArray::SharedPtr msg){
        if(!msg->arr.empty()){
            if(isCatchingNearest_){
                Turtle minTurtleDistance = msg->arr.at(0);
                double minDistance = getDistFromPose(minTurtleDistance);
                for(size_t i = 1 ; i < msg->arr.size();i++){
                    double currentDist = getDistFromPose(msg->arr.at(i));
                    if(currentDist < minDistance){
                        minDistance = currentDist;
                        minTurtleDistance = msg->arr.at(i);
                    }
                }
                target_ = minTurtleDistance;
            }
            else{
                target_ = msg->arr.at(0);    
            }    
        }
    }
    void callbackPose(const Pose::SharedPtr msg){
        position_ = *msg.get();
        isControllerUp_ = true;
    }
    void publishMoveCommand(){
        if(!isControllerUp_ || target_.name == ""){
            return;
        }
        try
        {
            double distX = target_.x - position_.x;
            double distY = target_.y - position_.y;
            double distance = std::sqrt(distX*distX + distY*distY);
            auto msg = Twist();
            
            if(distance > 0.5){
                msg.linear.x = 2*distance;

                double targetTheta = atan2(distY , distX);
                double diff = targetTheta - position_.theta;

                if(diff > M_PI)diff -= 2*M_PI;
                if(diff < -M_PI)diff += 2*M_PI;
                msg.angular.z = 6*diff;
            }
            else{
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
                catchTurtleThreads_.push_back(std::thread(std::bind(&SpawnerNode::callCatchService,this , target_.name)));
                target_.name = "";
            }
            publisher_->publish(msg);
        }
        catch(const char* e){
            RCLCPP_ERROR(this->get_logger() ,e);
        }
    }

    void callCatchService(const std::string &turtleName){
        auto client = this->create_client<CatchTurtle>("catch_service");
        while(!client->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_WARN(this->get_logger() , "waiting for the server to be up...");
        }
        auto request = std::make_shared<CatchTurtle::Request>();
        request->name = turtleName;
        auto future = client->async_send_request(request);
        try{
            auto response  = future.get();
            if(!response->success){
                RCLCPP_ERROR(this->get_logger() , (turtleName+ " cannot be caught").c_str());
            }    
        }
         catch (const std::exception &e){
             RCLCPP_ERROR(this->get_logger(), "service call failed: %s", e.what());
        }
    }

    //* class fields.
    bool isControllerUp_;
    bool isCatchingNearest_; 
    Turtle target_;
    Pose position_;
    rclcpp::Publisher<Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<Pose>::SharedPtr poseSubscriber_;
    rclcpp::Subscription<TurtleArray>::SharedPtr aliveTurtleSubscriber_;
    std::vector<std::thread> catchTurtleThreads_;
    
};

int main(int argc , char** argv){
    rclcpp::init(argc , argv);
    auto node = std::make_shared<SpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}