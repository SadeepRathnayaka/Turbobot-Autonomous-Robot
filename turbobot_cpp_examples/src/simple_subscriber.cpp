#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders:: _1 ;
const rclcpp::Logger LOGGER = rclcpp::get_logger("turbobot_cpp_examples");

class SimpleSubscriber : public rclcpp::Node
{
public :
    SimpleSubscriber() : Node("Simple_Subscriber")
    {
        sub_ = create_subscription<std_msgs::msg::String>("/chatter", 10, std::bind(&SimpleSubscriber::callBackfunction, this, _1));
    }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    void callBackfunction(const std_msgs::msg::String &msg) const
    {
        auto string_ = msg.data ;
        RCLCPP_INFO_STREAM(LOGGER, "New message received : " << string_);

    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  rclcpp::shutdown();
  return 0;
}