#include "food_del_robot_hardware/food_del_robot_hardware_interface.hpp"
#include <cmath>

namespace food_del_robot_hardware {

hardware_interface::CallbackReturn FoodDelRobotHardwareInterface::on_init
    (const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    can_port_ = "can0";
    can_bitrate_ = 500000;
    device_id_ = 0x01;

    // Initialize storage for 2 wheels
    position_states_.resize(2, 0.0);
    velocity_states_.resize(2, 0.0);
    velocity_commands_.resize(2, 0.0);

    driver_ = std::make_shared<WheelsDriver>(can_port_, can_bitrate_, device_id_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
FoodDelRobotHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    // Export state interfaces similar to old set_state/get_state pattern
    // Left wheel
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
            "base_left_wheel_joint", "velocity", &velocity_states_[0]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
            "base_left_wheel_joint", "position", &position_states_[0]));
    
    // Right wheel  
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
            "base_right_wheel_joint", "velocity", &velocity_states_[1]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
            "base_right_wheel_joint", "position", &position_states_[1]));
    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FoodDelRobotHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    
    // Export command interfaces
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
            "base_left_wheel_joint", "velocity", &velocity_commands_[0]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
            "base_right_wheel_joint", "velocity", &velocity_commands_[1]));
    
    return command_interfaces;
}

hardware_interface::CallbackReturn FoodDelRobotHardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    if (driver_->init() == false) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FoodDelRobotHardwareInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    // Initialize all values to 0 (replaces old set_state calls)
    velocity_states_[0] = 0.0;    // base_left_wheel_joint/velocity
    velocity_states_[1] = 0.0;    // base_right_wheel_joint/velocity
    position_states_[0] = 0.0;    // base_left_wheel_joint/position
    position_states_[1] = 0.0;    // base_right_wheel_joint/position
    velocity_commands_[0] = 0.0;  // base_left_wheel_joint/velocity command
    velocity_commands_[1] = 0.0;  // base_right_wheel_joint/velocity command
    
    driver_->set_velocity_mode();
    driver_->enable();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FoodDelRobotHardwareInterface::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    driver_->shutdown();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FoodDelRobotHardwareInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    float left_vel = 0.0f;
    float right_vel = 0.0f;

    if (!driver_->get_speed_rad_per_sec(left_vel, right_vel))
        return hardware_interface::return_type::ERROR;
    
    // Update state values (replaces set_state calls)
    velocity_states_[0] = -1.0f * left_vel;   // base_left_wheel_joint/velocity
    velocity_states_[1] = right_vel;   // base_right_wheel_joint/velocity
    
    // Integrate velocity to get position 
    position_states_[0] += left_vel * period.seconds();
    position_states_[1] += right_vel * period.seconds();
    
    RCLCPP_INFO(rclcpp::get_logger("food_del_robot_hardware_interface"), "left vel: %lf, right vel: %lf, left pos: %lf, right pos: %lf",
             left_vel, right_vel, position_states_[0], position_states_[1]);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FoodDelRobotHardwareInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    
    // Send commands to hardware (replaces get_command calls)
    driver_->set_sync_left_right_speed(velocity_commands_[0], velocity_commands_[1]);
    
    // RCLCPP_INFO(get_logger(), "left vel: %lf, right vel: %lf", velocity_commands_[0], 
    //                     velocity_commands_[1]);
    return hardware_interface::return_type::OK;
}

} // namespace food_del_robot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(food_del_robot_hardware::FoodDelRobotHardwareInterface, hardware_interface::SystemInterface)