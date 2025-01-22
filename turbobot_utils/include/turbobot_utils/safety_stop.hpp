#ifndef SAFETY_STOP_HPP
#define SAFETY_STOP_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"


enum State
{
    FREE = 0,
    WARNING = 1,
    DANGER = 2
};


class SafetyStop : public rclcpp::Node
{
public :
    SafetyStop(const std::string& name);

private:

    void laserCallback(const sensor_msgs::msg::LaserScan &);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_ ;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub_;

    double danger_distance_;
    std::string scan_topic_;
    std::string safety_stop_topic_;
    State state_;

};

#endif