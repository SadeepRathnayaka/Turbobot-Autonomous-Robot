#include "turbobot_controller/twist_relay.hpp"

TwistRelay::TwistRelay(const std::string& name) : Node(name)
{
    controller_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/turbobot_controller/cmd_vel_unstamped", 
        10,
        std::bind(&TwistRelay::controller_twist_callback, this, std::placeholders::_1));

    controller_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/turbobot_controller/cmd_vel", 10); 

    joy_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "input_joy/cmd_vel_stamped",
        10,
        std::bind(&TwistRelay::joy_twist_callback, this, std::placeholders::_1));

    joy_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "/input_joy/cmd_vel", 10 );

};

void TwistRelay::controller_twist_callback(const geometry_msgs::msg::Twist &msg)
{
    geometry_msgs::msg::TwistStamped twist_stamped_;
    twist_stamped_.header.stamp = get_clock()->now();
    twist_stamped_.twist = msg;
    controller_pub_->publish(twist_stamped_);
}

void TwistRelay::joy_twist_callback(const geometry_msgs::msg::TwistStamped &msg)
{   
    geometry_msgs::msg::Twist twist_;
    twist_ = msg.twist;
    joy_pub_->publish(twist_);
}

int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistRelay>("Twist_Relay");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}