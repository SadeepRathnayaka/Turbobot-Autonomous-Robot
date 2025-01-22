#include <math.h>
#include "turbobot_utils/safety_stop.hpp"

SafetyStop::SafetyStop(const std::string& name) : Node(name), state_{State::FREE}
{   

    declare_parameter<double>("danger_distance", 0.2);
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("safety_stop_topic", "/safety_stop");

    danger_distance_  = get_parameter("danger_distance").as_double();
    scan_topic_       = get_parameter("scan_topic").as_string();
    safety_stop_topic_= get_parameter("safety_stop_topic").as_string();

    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, 10, std::bind(&SafetyStop::laserCallback, this, std::placeholders::_1));

    safety_stop_pub_ = create_publisher<std_msgs::msg::Bool>(safety_stop_topic_, 10);
}

void SafetyStop::laserCallback(const sensor_msgs::msg::LaserScan& msg)
{   
    state_ = State::FREE;
    for (const auto & range : msg.ranges)
    {
        if (!std::isinf(range) && range <= danger_distance_)
        {   
            RCLCPP_INFO(rclcpp::get_logger("Safety_Stop"), "Robot is in danger zone. Stopping ...");
            state_ = State::DANGER ;
            break;
        }
    }

    std_msgs::msg::Bool is_safety_stop_ ;

    if (state_ == State::DANGER) is_safety_stop_.data = true ;
    else if (state_ == State::FREE) is_safety_stop_.data = false ;

    safety_stop_pub_->publish(is_safety_stop_);

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetyStop>("safety_stop_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}