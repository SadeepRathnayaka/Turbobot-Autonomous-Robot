#include "turbobot_firmware/turbobot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace  turbobot_firmware
{

const rclcpp::Logger LOGGER = rclcpp::get_logger("TurbobotInterface");

TurbobotInterface::TurbobotInterface() {}

TurbobotInterface::~TurbobotInterface()
{
    if (arduino_.IsOpen())
    {
        try
        {
            arduino_.Close();
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(LOGGER, "Something went wrong while closing connection with port " << port_);
        }
        
    }
}

CallbackReturn TurbobotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    try
    {
        port_ = info_.hardware_parameters.at("port");
    }
    catch(const std::out_of_range &e)
    {
        RCLCPP_FATAL(LOGGER, "No Serial Port provided! Aborting");
        return CallbackReturn::FAILURE;
    }

    velocity_commands_.reserve(info_.joints.size());
    velocity_states_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
    last_run_ = rclcpp::Clock().now();

    return CallbackReturn::SUCCESS;
    
}

std::vector<hardware_interface::StateInterface> TurbobotInterface::export_state_interfaces() 
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
    }

    return state_interfaces ;
}

std::vector<hardware_interface::CommandInterface> TurbobotInterface::export_command_interfaces() 
{
    std::vector<hardware_interface::CommandInterface> command_interfaces; 

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
    }

    return command_interfaces;
}

CallbackReturn TurbobotInterface::on_activate(const rclcpp_lifecycle::State &) 
{
    RCLCPP_INFO(LOGGER, "Starting the Robot Hardware ... ");

    velocity_commands_ = {0.0, 0.0, 0.0, 0.0};     // left front, left back, right front, right back
    velocity_states_   = {0.0, 0.0, 0.0, 0.0};      
    position_states_   = {0.0, 0.0, 0.0, 0.0};

    try
    {
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch(...)
    {   
        RCLCPP_FATAL_STREAM(LOGGER, "Something went wrong while openning the connection with port " << port_);
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(LOGGER, "Hardware started successfully, waiting for commands ... ");
    return CallbackReturn::SUCCESS;
    
}
CallbackReturn TurbobotInterface::on_deactivate(const rclcpp_lifecycle::State &) 
{
    RCLCPP_INFO(LOGGER, "Stopping the Robot Hardware ... ");

    try
    {
        arduino_.Close();
    }
    catch(...)
    {
        RCLCPP_FATAL_STREAM(LOGGER, "Something went wrong while closing the connection with port " << port_);
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(LOGGER, "Hardware stopped successfully!");
    return CallbackReturn::SUCCESS;
    
}

hardware_interface::return_type TurbobotInterface::read(const rclcpp::Time &, const rclcpp::Duration &) 
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
            multiplier = res.at(1) == 'p' ? 1 : -1 ;

            if (res.at(0) == 'l')
            {
                velocity_states_.at(0)  = multiplier * std::stod(res.substr(2, res.size()));
                velocity_states_.at(1)  = multiplier * std::stod(res.substr(2, res.size()));
                position_states_.at(0) += multiplier * std::stod(res.substr(2, res.size())) * dt;
                position_states_.at(1) += multiplier * std::stod(res.substr(2, res.size())) * dt;
            }

            else if (res.at(0) == 'r')
            {
                velocity_states_.at(2)  = multiplier * std::stod(res.substr(2, res.size()));
                velocity_states_.at(3)  = multiplier * std::stod(res.substr(2, res.size()));
                position_states_.at(2) += multiplier * std::stod(res.substr(2, res.size())) * dt;
                position_states_.at(3) += multiplier * std::stod(res.substr(2, res.size())) * dt;
            }

            last_run_ = rclcpp::Clock().now();  
        }
    }
    return hardware_interface::return_type::OK;
}


hardware_interface::return_type TurbobotInterface::write(const rclcpp::Time &, const rclcpp::Duration &) 
{
    std::stringstream message_stream ;
    char left_wheel_sign  = velocity_commands_.at(0) >= 0 ? 'p' : 'n' ;
    char right_wheel_sign = velocity_commands_.at(2) >= 0 ? 'p' : 'n' ;

    float left_wheel_velocity  = (velocity_commands_.at(0) + velocity_commands_.at(1)) / 2 ; 
    float right_wheel_velocity = (velocity_commands_.at(2) + velocity_commands_.at(3)) / 2 ;

    std::string componsate_zeros_left  = "";
    std::string componsate_zeros_right = "";

    if (std::abs(left_wheel_velocity) < 10.0) 
    {
        componsate_zeros_left = "0";
    }
    else
    {
        componsate_zeros_left = "";
    }
    if (std::abs(right_wheel_velocity) < 10.0) 
    {
        componsate_zeros_right = "0";
    }
    else
    {
        componsate_zeros_right = "";
    }

    message_stream << std::fixed << std::setprecision(2) <<
    "l" << left_wheel_sign  << componsate_zeros_left  << std::abs(left_wheel_velocity)  << "," <<
    "r" << right_wheel_sign << componsate_zeros_right << std::abs(right_wheel_velocity) << "," ;

    try
    {
        arduino_.Write(message_stream.str());
    }
    catch(...)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Something went wrong when sending the command" << 
                            message_stream.str() << " to the port " << port_) ;
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}
} // namespace  turbobot_firmware

PLUGINLIB_EXPORT_CLASS(turbobot_firmware::TurbobotInterface, hardware_interface::SystemInterface)
