#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"

using my_robot_interfaces::msg::Turtle;
using my_robot_interfaces::msg::TurtleArray;
using my_robot_interfaces::srv::CatchTurtle;
using turtlesim::srv::Spawn;
using turtlesim::srv::Kill;
using std::placeholders::_1;
using std::placeholders::_2;

class SpawnerNode : public rclcpp::Node{
public:
    SpawnerNode(): Node("spawner_node") , turtleCounter_(1){
        this->declare_parameter("spawn_speed" , 1);
        spawnSpeed_ = this->get_parameter("spawn_speed").as_int();

        aliveTurtlePublisher_ = this->create_publisher<TurtleArray>("alive_turtles" , 10);
        spawnTimer_ = this->create_wall_timer(std::chrono::seconds(spawnSpeed_) , std::bind(&SpawnerNode::spawnNewTurtle, this));
        // publishTimer_ = this->create_wall_timer(std::chrono::microseconds(10) , std::bind(&SpawnerNode::publishAliveTurtle, this));
        catchServer_ = this->create_service<CatchTurtle>("catch_service" ,std::bind(&SpawnerNode::callbackCatchService, this , _1 , _2));
        RCLCPP_INFO(this->get_logger() , "spawner was started");
    }
private:
    void callbackCatchService(const CatchTurtle::Request::SharedPtr &request , const CatchTurtle::Response::SharedPtr &response){
        for(size_t i= 0;i < turtles_.size();i++){
            if(turtles_.at(i).name == request->name){
                killThreads_.push_back(std::thread(std::bind(&SpawnerNode::callKillService , this ,request->name)));
                break;
            }
        }
        response->success = true;
        
    }
    double randomDouble()
    {
        return double(std::rand()) / (double(RAND_MAX) + 1.0);
    }
    void spawnNewTurtle(){
        turtleCounter_ += 1;
        std::string name = "turtle" + std::to_string(turtleCounter_);
        double x = randomDouble()*10.0;
        double y = randomDouble()*10.0;
        double theta = (double(std::rand()/(double(RAND_MAX)) + 1.0))*2*M_PI;
        spawnThreads_.push_back(std::thread(std::bind(&SpawnerNode::callSpawnService ,this ,name,x,y,theta)));

    }

    void callSpawnService(const std::string &name , double x , double y , double theta){
        auto client = this->create_client<Spawn>("spawn");
        while(!client->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_WARN(this->get_logger() , "waiting for the server to be up...");
        }
        auto request = std::make_shared<Spawn::Request>();
        request->name = name;
        request->x = x;
        request->y = y;
        request->theta = theta;
        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            if(response->name != ""){
                RCLCPP_INFO(this->get_logger() , "%s spawned successfully", response->name.c_str());
                auto turtle = Turtle();
                turtle.name = response->name;
                turtle.x = x;
                turtle.y = y;
                turtle.theta = theta;
                turtles_.push_back(turtle);
                publishAliveTurtle();
                
            }    
        }
        catch (const std::exception &e){
             RCLCPP_ERROR(this->get_logger(), "service call failed: %s", e.what());
        }
    }

    void callKillService(const std::string &name){
        auto client = this->create_client<Kill>("kill");
        while(!client->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_WARN(this->get_logger() , "waiting for the server to be up...");
        }
        auto request = std::make_shared<Kill::Request>();
        request->name = name;
        auto future = client->async_send_request(request);

        try{
            auto response = future.get();
            for(size_t i = 0;i < turtles_.size();i++){
                if(turtles_.at(i).name == name){
                    turtles_.erase(turtles_.begin() + i);
                    publishAliveTurtle();
                    break;
                }
            }
        }
        catch (const std::exception &e){
             RCLCPP_ERROR(this->get_logger(), "service call failed: %s", e.what());
        }
    }

    void publishAliveTurtle(){
        auto msg = TurtleArray();
        msg.arr = turtles_;

        aliveTurtlePublisher_->publish(msg);
    }


    int spawnSpeed_;
    std::vector<Turtle> turtles_;
    int turtleCounter_;
    rclcpp::Publisher<TurtleArray>::SharedPtr aliveTurtlePublisher_;
    rclcpp::TimerBase::SharedPtr spawnTimer_;
    rclcpp::TimerBase::SharedPtr publishTimer_;
    rclcpp::Service<CatchTurtle>::SharedPtr catchServer_;
    std::vector<std::thread> spawnThreads_;
    std::vector<std::thread> killThreads_;
};

int main(int argc , char** argv){
    rclcpp::init(argc , argv);
    auto node = std::make_shared<SpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}