#include "mi_ackermann_hardware/ackermann_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mi_ackermann_hardware {

// ─────────────────────────────────────────────────────────────────────────────
// on_init()
// Lee los parámetros del URDF. Todavía no abre conexión con el EV3.
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn AckermannHardware::on_init
    (const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    info_ = info;

    ip_   = info_.hardware_parameters["ip"];
    port_ = std::stoi(info_.hardware_parameters["port"]);

    std::string motor_type = info_.hardware_parameters["traction_type"];
    traction_type_ = (motor_type == "large") ? EV3MotorType::LARGE : EV3MotorType::MEDIUM;

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_configure()
// UNCONFIGURED → INACTIVE
// Abre la conexión TCP con el EV3.
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn AckermannHardware::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    driver_ = std::make_unique<EV3Driver>(ip_, port_, traction_type_);

    if (driver_->init() != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("AckermannHardware"),
            "No se pudo conectar al EV3 en %s:%d", ip_.c_str(), port_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("AckermannHardware"),
        "Conexion TCP establecida con el EV3.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_activate()
// INACTIVE → ACTIVE
// Inicializa variables directamente — no usar set_state() aquí.
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn AckermannHardware::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    traccion_pos_state_  = 0.0;
    traccion_vel_state_  = 0.0;
    steering_pos_state_  = 0.0;
    steering_vel_state_  = 0.0;
    traccion_vel_cmd_    = 0.0;
    steering_pos_cmd_    = 0.0;

    RCLCPP_INFO(rclcpp::get_logger("AckermannHardware"),
        "Hardware activado — robot listo para recibir comandos.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_deactivate()
// ACTIVE → INACTIVE
// Detiene el robot.
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn AckermannHardware::on_deactivate
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    driver_->setTractionVelocity(0.0);
    driver_->setSteeringPosition(0.0);
    driver_->sendAndReceive();

    RCLCPP_INFO(rclcpp::get_logger("AckermannHardware"),
        "Hardware desactivado — robot detenido.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_cleanup()
// INACTIVE → UNCONFIGURED
// Cierra la conexión TCP.
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn AckermannHardware::on_cleanup
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    if (driver_) {
        driver_->close();
        driver_.reset();
    }

    RCLCPP_INFO(rclcpp::get_logger("AckermannHardware"),
        "Conexion cerrada.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_shutdown()
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn AckermannHardware::on_shutdown
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    if (driver_ && driver_->isConnected()) {
        driver_->setTractionVelocity(0.0);
        driver_->setSteeringPosition(0.0);
        driver_->sendAndReceive();
        driver_->close();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// on_error()
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn AckermannHardware::on_error
    (const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    RCLCPP_ERROR(rclcpp::get_logger("AckermannHardware"),
        "Error en el hardware — cerrando conexion con el EV3.");

    if (driver_) {
        driver_->close();
        driver_.reset();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_state_interfaces()
// Los punteros a las variables permiten que el RM las lea directamente.
// Los nombres tienen que coincidir EXACTAMENTE con los del URDF.
// ─────────────────────────────────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface>
AckermannHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;

    interfaces.emplace_back("virtual_rear_wheel_joint",
        hardware_interface::HW_IF_POSITION, &traccion_pos_state_);
    interfaces.emplace_back("virtual_rear_wheel_joint",
        hardware_interface::HW_IF_VELOCITY, &traccion_vel_state_);
    interfaces.emplace_back("virtual_front_wheel_joint",
        hardware_interface::HW_IF_POSITION, &steering_pos_state_);
    interfaces.emplace_back("virtual_front_wheel_joint",
        hardware_interface::HW_IF_VELOCITY, &steering_vel_state_);

    return interfaces;
}

// ─────────────────────────────────────────────────────────────────────────────
// export_command_interfaces()
// ─────────────────────────────────────────────────────────────────────────────
std::vector<hardware_interface::CommandInterface>
AckermannHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;

    interfaces.emplace_back("virtual_rear_wheel_joint",
        hardware_interface::HW_IF_VELOCITY, &traccion_vel_cmd_);
    interfaces.emplace_back("virtual_front_wheel_joint",
        hardware_interface::HW_IF_POSITION, &steering_pos_cmd_);

    return interfaces;
}

// ─────────────────────────────────────────────────────────────────────────────
// read()
// Corre cada ciclo — lee encoders del EV3 y actualiza las variables.
// El RM lee las variables directamente via los punteros exportados.
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type AckermannHardware::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;

    double vel = - driver_->getTractionVelocityRadS();

    // filtro de ruido — evita movimiento fantasma por lecturas pequeñas
    if (std::abs(vel) < 0.03) { vel = 0.0; }

    traccion_vel_state_  = vel;
    traccion_pos_state_ += vel * period.seconds();
    steering_pos_state_  = driver_->getSteeringPositionRad();
    steering_vel_state_  = 0.0;

    return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// write()
// Corre cada ciclo — manda comandos al EV3.
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type AckermannHardware::write
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    driver_->setTractionVelocity(traccion_vel_cmd_);
    driver_->setSteeringPosition(steering_pos_cmd_);

    if (!driver_->sendAndReceive()) {
        RCLCPP_ERROR(rclcpp::get_logger("AckermannHardware"),
            "Error de comunicacion con el EV3.");
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

} // namespace mi_ackermann_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    mi_ackermann_hardware::AckermannHardware,
    hardware_interface::SystemInterface
)