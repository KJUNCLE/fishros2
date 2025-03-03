#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "chapter4_interfaces/srv/patrol.hpp"
using Patrol = chapter4_interfaces::srv::Patrol;

class TurtleController:public rclcpp::Node
{
    public:
        TurtleController() : Node("turtle_controller")
        {
            patrol_service_ = this->create_service<Patrol>("patrol",[&](const 
                Patrol::Request::SharedPtr request,Patrol::Response::SharedPtr response) -> void
            {
                if ((0<request->target_x && request->target_x<12)&&
                    (0<request->target_y && request->target_y<12))
                {
                    this -> target_x_ = request->target_x;
                    this -> target_y_ = request->target_y;
                    response->result = Patrol::Response::SUCCESS;
                }
                else
                {
                    response->result = Patrol::Response::FAIL;
                }
            });

            velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "/turtle1/cmd_vel",10);
            pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
                    "/turtle1/pose",10,
                    std::bind(&TurtleController::on_pose_received_, this, std::placeholders::_1));
        }

    private:
        void on_pose_received_(const turtlesim::msg::Pose::SharedPtr pose) const
        {
            auto message = geometry_msgs::msg::Twist();
            // 1.记录当前位置
            double current_x = pose->x;
            double current_y = pose->y;
            RCLCPP_INFO(this->get_logger(),"当前位置：（x=%f,y=%f）",current_x,current_y);

            // 2.计算距离目标的距离，与当前海归朝向的角度差
            double distance = sqrt((target_x_ - current_x) * (target_x_ - current_x) + 
                                    (target_y_ - current_y) * (target_y_ - current_y));
            double angle = atan2(target_y_ - current_y, target_x_ - current_x) - pose->theta;

            
            // 3.根据距离与角度差来控制海龟的移动
            if (distance > 0.01){
                if(fabs(angle) > 0.2)
                {
                    message.angular.z = fabs(angle);
                }
                else
                {
                    message.linear.x = distance;
                }
            }
            
            // 4.限制最大值并发布速度
            if (message.linear.x > max_speed_)
            {
                message.linear.x = max_speed_;
            }
            velocity_publisher_->publish(message);
        }
    private:
        rclcpp::Service<Patrol>::SharedPtr patrol_service_; 
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
        double target_x_{1.0};
        double target_y_{1.0};
        double k_{1.0};
        double max_speed_{3.0};
};

int main(int argc , char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}