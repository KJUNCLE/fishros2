#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
using namespace std::chrono_literals;

class TurtleCircle : public rclcpp::Node
{
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    public:
        explicit TurtleCircle(const std::string& node_name):Node(node_name)
        {
            // 调用继承而来的父类函数创建订阅者
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel",10);
            // 调用继承而来的父类函数创建定时器
            timer_ = this->create_wall_timer(1000ms, std::bind(&TurtleCircle::timer_callback, this));
        }
    
        private:
        void timer_callback()
        {
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = 1.0;
            msg.angular.z = 0.5;
            publisher_->publish(msg);
        }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleCircle>("turtle_circle"));
    rclcpp::shutdown();
    return 0;
}

