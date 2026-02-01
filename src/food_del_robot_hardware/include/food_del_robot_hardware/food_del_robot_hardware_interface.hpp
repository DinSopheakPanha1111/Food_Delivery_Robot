#ifndef FOOD_DEL_ROBOT_HARDWARE_INTERFACE_HPP
#define FOOD_DEL_ROBOT_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "food_del_robot_hardware/wheels_driver.hpp"
#include "rclcpp/rclcpp.hpp"

namespace food_del_robot_hardware {

class FoodDelRobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
    // Lifecycle node override
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // SystemInterface override
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface>
        export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface>
        export_command_interfaces() override;
    hardware_interface::return_type
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::shared_ptr<WheelsDriver> driver_;
    std::string can_port_;
    int can_bitrate_;
    int device_id_;
    
    // Storage for state and command values
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    std::vector<double> velocity_commands_;

}; // class FoodDelRobotHardwareInterface

} // namespace food_del_robot_hardware

#endif