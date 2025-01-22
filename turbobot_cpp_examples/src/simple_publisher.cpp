#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>

using namespace std::chrono_literals;


const rclcpp::Logger LOGGER = rclcpp::get_logger("turbobot_cpp_examples");

class SimplePublisher : public rclcpp::Node 
{
public:
    SimplePublisher() : Node("Simple_Publisher"), counter_(0)
    {
        pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
        timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));

        RCLCPP_INFO(LOGGER, "Simple Publisher node started. Publishing at 1 Hz frequency... ");
    }

private:
    unsigned int counter_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_ ;
    rclcpp::TimerBase::SharedPtr timer_ ; 

    void timerCallback()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello ROS2 - counter" + std::to_string(counter_++);
        // counter_++ ;

        pub_->publish(msg);
    }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}