#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <chrono>

using namespace std::chrono_literals;

class CmdVelInterpolator : public rclcpp::Node
{
public:
    CmdVelInterpolator()
        : Node("cmd_vel_interpolator"), interpolation_steps_(10)
    {
        // Declare and get parameters
        this->declare_parameter<int>("interpolation_steps", 10);
        interpolation_steps_ = this->get_parameter("interpolation_steps").as_int();

        // Subscriber and Publisher
        cmd_vel_unstamped_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/turbobot_controller/cmd_vel_unstamped", 10,
            std::bind(&CmdVelInterpolator::cmdVelCallback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/turbobot_controller/cmd_vel", 10);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Interpolate and publish
        geometry_msgs::msg::Twist previous = last_twist_;
        geometry_msgs::msg::Twist target = *msg;

        for (int i = 1; i <= interpolation_steps_; ++i)
        {
            auto interpolated_twist = interpolateTwist(previous, target, i);

            geometry_msgs::msg::TwistStamped stamped_msg;
            stamped_msg.header.stamp = this->now();
            stamped_msg.twist = interpolated_twist;

            cmd_vel_pub_->publish(stamped_msg);
            rclcpp::sleep_for(5ms); // Adjust timing as needed
        }

        last_twist_ = target; // Update the last twist to the current target
    }

    geometry_msgs::msg::Twist interpolateTwist(
        const geometry_msgs::msg::Twist &start, const geometry_msgs::msg::Twist &end, int step)
    {
        geometry_msgs::msg::Twist result;

        double ratio = static_cast<double>(step) / interpolation_steps_;
        result.linear.x = start.linear.x + ratio * (end.linear.x - start.linear.x);
        result.linear.y = start.linear.y + ratio * (end.linear.y - start.linear.y);
        result.linear.z = start.linear.z + ratio * (end.linear.z - start.linear.z);

        result.angular.x = start.angular.x + ratio * (end.angular.x - start.angular.x);
        result.angular.y = start.angular.y + ratio * (end.angular.y - start.angular.y);
        result.angular.z = start.angular.z + ratio * (end.angular.z - start.angular.z);

        return result;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_unstamped_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

    geometry_msgs::msg::Twist last_twist_;
    int interpolation_steps_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelInterpolator>());
    rclcpp::shutdown();
    return 0;
}
