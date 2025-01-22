#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>

#include <libserial/SerialPort.h>

using namespace std::chrono_literals;


class SimpleSerialReceiver : public rclcpp::Node
{
public:
  SimpleSerialReceiver() : Node("simple_serial_receiver")
  {
    declare_parameter<std::string>("port", "/dev/ttyUSB0");

    port_ = get_parameter("port").as_string();

    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

    pub_ = create_publisher<std_msgs::msg::String>("serial_receiver", 10);
    timer_ = create_wall_timer(0.01s, std::bind(&SimpleSerialReceiver::timerCallback, this));

  }

  ~SimpleSerialReceiver()
  {
    arduino_.Close();
  }

  void timerCallback()
  {
    if (arduino_.IsDataAvailable())
    {
      auto dt = (rclcpp::Clock().now() - last_run_).seconds();
      std::string message;
      arduino_.ReadLine(message);
      std::stringstream ss(message);
      std::string res;
      int multiplier = 1;

      while (std::getline(ss, res, ','))
      {
        multiplier = res.at(1) == 'p' ? 1 : -1;

        if (res.at(0) == 'l')
        {
          velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
          velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
          position_states_.at(0) += velocity_states_.at(0) * dt;
          position_states_.at(1) += velocity_states_.at(0) * dt;
        }
        else if (res.at(0) == 'r')
        {
          velocity_states_.at(2) = multiplier * std::stod(res.substr(2, res.size()));
          velocity_states_.at(3) = multiplier * std::stod(res.substr(2, res.size()));
          position_states_.at(2) += velocity_states_.at(1) * dt;
          position_states_.at(3) += velocity_states_.at(1) * dt;
        }
      }

      // Convert the states to a string for publishing
      std::stringstream feedback_stream;
      feedback_stream << "Left Front Wheel - Velocity: " << velocity_states_.at(0)
                      << ", Position: " << position_states_.at(0) << "\n"
                     << "Left Back Wheel - Velocity: " << velocity_states_.at(1)
                     << ", Position: " << position_states_.at(1) << "\n"
                     << "Right Front Wheel - Velocity: " << velocity_states_.at(2)
                     << ", Position: " << position_states_.at(2) << "\n"
                     << "Right Back Wheel - Velocity: " << velocity_states_.at(3)
                     << ", Position: " << position_states_.at(3) << "\n" ;
                     

      // Prepare the string message
      auto msg = std_msgs::msg::String();
      msg.data = feedback_stream.str();

      // Publish the feedback
      pub_->publish(msg);

      last_run_ = rclcpp::Clock().now();
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string port_;
  LibSerial::SerialPort arduino_;
  rclcpp::Time last_run_;
  std::vector<double> velocity_states_ = {0.0,0.0,0.0,0.0};
  std::vector<double> position_states_ = {0.0,0.0,0.0,0.0};

};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleSerialReceiver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}