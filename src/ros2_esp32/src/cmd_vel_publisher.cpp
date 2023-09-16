#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/Twist.h>

class Cmd_Vel_Publisher : public rclcpp::Node
{
public:
    CmdVelPublisher() : Node("send_data")
    {
        publisher_ = this->create_publisher<geometry_msgs::Twist>("cmd_vel", 15);

        timer_ = this->create_wall_timer(std::chrono::seconds(0.2),
                                         std::bind(&CmdVelPublisher::publishData, this));
    }

private:
    void publishData()
    {
        auto msg = ;
        
        
        RCLCPP_INFO(this->get_logger(), "%i", msg.data);
        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::Twist>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}