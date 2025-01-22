#ifndef TWIST_RELAY_HPP
#define TWIST_RELAY_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistRelay : public rclcpp::Node
{
public :
    TwistRelay(const std::string& name);

private:

    void controller_twist_callback(const geometry_msgs::msg::Twist & );
    void joy_twist_callback(const geometry_msgs::msg::TwistStamped & );

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controller_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr controller_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joy_pub_;
};


#endif