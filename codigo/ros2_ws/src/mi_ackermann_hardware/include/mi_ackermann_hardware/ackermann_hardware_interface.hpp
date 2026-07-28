#ifndef MI_ACKERMANN_HARDWARE__ACKERMANN_HARDWARE_INTERFACE_HPP_
#define MI_ACKERMANN_HARDWARE__ACKERMANN_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "mi_ackermann_hardware/ev3_driver.hpp"

namespace mi_ackermann_hardware {

class AckermannHardware : public hardware_interface::SystemInterface
{
public:

    // ── Ciclo de vida ─────────────────────────────────────────────────────────
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn
        on_error(const rclcpp_lifecycle::State & previous_state) override;

    // ── Interfaces ───────────────────────────────────────────────────────────
    std::vector<hardware_interface::StateInterface>
        export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
        export_command_interfaces() override;

    // ── Loop de control ──────────────────────────────────────────────────────
    hardware_interface::return_type
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

    // ── Driver TCP al EV3 ────────────────────────────────────────────────────
    std::unique_ptr<EV3Driver> driver_;

    // ── Parámetros leídos del URDF ───────────────────────────────────────────
    std::string  ip_;
    int          port_;
    EV3MotorType traction_type_;

    // ── State interfaces — lo que el controller LEE ──────────────────────────
    double traccion_pos_state_  = 0.0;   // virtual_rear_wheel_joint/position
    double traccion_vel_state_  = 0.0;   // virtual_rear_wheel_joint/velocity
    double steering_pos_state_  = 0.0;   // virtual_front_wheel_joint/position
    double steering_vel_state_  = 0.0;   // virtual_front_wheel_joint/velocity

    // ── Command interfaces — lo que el controller ESCRIBE ────────────────────
    double traccion_vel_cmd_    = 0.0;   // virtual_rear_wheel_joint/velocity
    double steering_pos_cmd_    = 0.0;   // virtual_front_wheel_joint/position

}; // class AckermannHardware

} // namespace mi_ackermann_hardware

#endif  // MI_ACKERMANN_HARDWARE__ACKERMANN_HARDWARE_INTERFACE_HPP_
