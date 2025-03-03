#include "rclcpp/rclcpp.hpp"
#include "chapter4_interfaces/srv/patrol.hpp"
#include <chrono>
using Patrol = chapter4_interfaces::srv::Patrol;
using namespace std::chrono_literals;


class PatrolClient:public rclcpp::Node
{
    public:
        PatrolClient() : Node("patrol_client")
        {
            srand(time(NULL)); // 初始化随机数种子
            patrol_client_ = this->create_client<Patrol>("patrol");
            timer_ = this->create_wall_timer(10s, std::bind(&PatrolClient::timer_callback, this));
        }

        void timer_callback()
        {
            while (!patrol_client_->wait_for_service(std::chrono::seconds(1)))
            {
                // 等待时检测rclcpp的状态
                if(!rclcpp::ok()){
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
            RCLCPP_INFO(this->get_logger(), "Patrol service not available, waiting again...");
            }
            // 2. 构造请求对象
            auto request = std::make_shared<Patrol::Request>();
            request->target_x = rand() % 15;
            request->target_y = rand() % 15;
            RCLCPP_INFO(this->get_logger(),"Request(%f,%f)",request->target_x, request->target_y);
            
            // 3. 发送请求
            this->patrol_client_ ->async_send_request(
                request,
                [&](rclcpp::Client<Patrol>::SharedFuture result_future) -> void{
                    auto response = result_future.get();
                    if(response->result == Patrol::Response::SUCCESS){
                        RCLCPP_INFO(this->get_logger(), "success");
                    }
                    else if (response->result == Patrol::Response::FAIL){
                        RCLCPP_INFO(this->get_logger(), "fail");
                    }
                });
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Client<Patrol>::SharedPtr patrol_client_;
};

int main(int argc , char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PatrolClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}