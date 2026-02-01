#include "food_del_robot_hardware/food_del_robot_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
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

    // Resize based on actual number of joints from URDF
    size_t num_joints = info_.joints.size();
    position_states_.resize(num_joints, 0.0);
    velocity_states_.resize(num_joints, 0.0);
    velocity_commands_.resize(num_joints, 0.0);

    RCLCPP_INFO(rclcpp::get_logger("FoodDelRobotHardwareInterface"),
                "Initializing hardware interface with %zu joints", num_joints);

    // Create CAN and Driver objects
    can_ = std::make_shared<CAN>("can0", 500000);
    driver_ = std::make_shared<ZLAC8015DDriver>(*can_, 0x01);

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
FoodDelRobotHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    RCLCPP_INFO(rclcpp::get_logger("FoodDelRobotHardwareInterface"), 
                "Exporting state interfaces for %zu joints", info_.joints.size());

    for (size_t i = 0; i < info_.joints.size(); i++) {
        // Velocity state interface
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &velocity_states_[i]
        ));
        
        // Position state interface
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &position_states_[i]
        ));
        
        RCLCPP_INFO(rclcpp::get_logger("FoodDelRobotHardwareInterface"), 
                    "Added state interfaces for joint %s: velocity, position",
                    info_.joints[i].name.c_str());
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FoodDelRobotHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    RCLCPP_INFO(rclcpp::get_logger("FoodDelRobotHardwareInterface"), 
                "Exporting command interfaces for %zu joints", info_.joints.size());

    for (size_t i = 0; i < info_.joints.size(); i++) {
        // Velocity command interface
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &velocity_commands_[i]
        ));
        
        RCLCPP_INFO(rclcpp::get_logger("FoodDelRobotHardwareInterface"), 
                    "Added command interface for joint %s: velocity",
                    info_.joints[i].name.c_str());
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn FoodDelRobotHardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    
    // Enable CAN interface
    can_->enable();
    
    RCLCPP_INFO(rclcpp::get_logger("FoodDelRobotHardwareInterface"),
                "CAN interface configured successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FoodDelRobotHardwareInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    
    // Initialize all values to 0
    for (size_t i = 0; i < info_.joints.size(); i++) {
        velocity_states_[i] = 0.0;
        position_states_[i] = 0.0;
        velocity_commands_[i] = 0.0;
    }
    
    // Initialize driver
    if (!driver_->set_velocity_mode()) {
        RCLCPP_ERROR(rclcpp::get_logger("FoodDelRobotHardwareInterface"),
                     "Failed to set velocity mode");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    if (!driver_->enable()) {
        RCLCPP_ERROR(rclcpp::get_logger("FoodDelRobotHardwareInterface"),
                     "Failed to enable driver");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("FoodDelRobotHardwareInterface"),
                "Hardware activated successfully");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FoodDelRobotHardwareInterface::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    
    if (!driver_->emergency_stop()) {
        RCLCPP_WARN(rclcpp::get_logger("FoodDelRobotHardwareInterface"),
                    "Failed to emergency stop driver");
    }
    
    RCLCPP_INFO(rclcpp::get_logger("FoodDelRobotHardwareInterface"),
                "Hardware deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FoodDelRobotHardwareInterface::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;

    float left_vel_rpm = 0.0f;
    float right_vel_rpm = 0.0f;

    bool read_ok = driver_->read_speed_feedback(left_vel_rpm, right_vel_rpm);

    if (!read_ok) {
        // === Simulate / fake feedback when no real motors are connected ===
        // Option A: Use commanded velocity as fake measured velocity (open-loop)
        // (most common for desk testing)
        left_vel_rpm  = velocity_commands_[0]  / 0.104719755f;  // rad/s → RPM
        right_vel_rpm = velocity_commands_[1] / 0.104719755f;

        // Option B: Keep zero if you prefer (but then odometry stays still)
        // left_vel_rpm = 0.0f; right_vel_rpm = 0.0f;

        RCLCPP_WARN_ONCE(rclcpp::get_logger("FoodDelRobotHardwareInterface"),
            "No real feedback available → faking states from commands (open-loop mode)");
    }

    // Convert RPM → rad/s
    const float RPM_TO_RAD_PER_SEC = 0.104719755f;
    float left_vel_rad  = left_vel_rpm  * RPM_TO_RAD_PER_SEC;
    float right_vel_rad = right_vel_rpm * RPM_TO_RAD_PER_SEC;

    velocity_states_[0] = -left_vel_rad;   // inverted if needed
    velocity_states_[1] =  right_vel_rad;

    // Integrate position (always do this — even in simulation)
    position_states_[0] += velocity_states_[0] * period.seconds();
    position_states_[1] += velocity_states_[1] * period.seconds();

    // Your debug print...
    static int counter = 0;
    if (counter++ % 100 == 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("FoodDelRobotHardwareInterface"), 
                     "Read: left_vel=%f rad/s, right_vel=%f rad/s, left_pos=%f, right_pos=%f",
                     velocity_states_[0], velocity_states_[1], 
                     position_states_[0], position_states_[1]);
    }

    return hardware_interface::return_type::OK;   // ← always OK now
}

hardware_interface::return_type FoodDelRobotHardwareInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    
    // Convert rad/s to RPM
    // 1 rad/s = (60/2π) RPM = 9.54929658551 RPM
    const float RAD_PER_SEC_TO_RPM = 9.54929658551f;
    
    // Get velocity commands for both wheels
    float left_vel_rad_per_sec = velocity_commands_[0];  // Left wheel command
    float right_vel_rad_per_sec = velocity_commands_[1]; // Right wheel command
    
    // Convert to RPM
    float left_rpm = left_vel_rad_per_sec * RAD_PER_SEC_TO_RPM;
    float right_rpm = right_vel_rad_per_sec * RAD_PER_SEC_TO_RPM;
    
    // Send commands to hardware
    // Note: Left wheel might need to be inverted (-1.0f) depending on your motor orientation
    if (!driver_->set_sync_left_right_speed(-left_rpm, right_rpm)) {
        RCLCPP_WARN(rclcpp::get_logger("FoodDelRobotHardwareInterface"),
                    "Failed to set wheel speeds");
        return hardware_interface::return_type::ERROR;
    }
    
    // Debug logging (reduce frequency)
    static int counter = 0;
    if (counter++ % 100 == 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("FoodDelRobotHardwareInterface"), 
                     "Write: left_cmd=%f rad/s (%f RPM), right_cmd=%f rad/s (%f RPM)",
                     left_vel_rad_per_sec, left_rpm,
                     right_vel_rad_per_sec, right_rpm);
    }
    
    return hardware_interface::return_type::OK;
}

} // namespace food_del_robot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(food_del_robot_hardware::FoodDelRobotHardwareInterface, hardware_interface::SystemInterface)