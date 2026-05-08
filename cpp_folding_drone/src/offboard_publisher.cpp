/**
 * offboard_publisher.cpp
 *
 * Arms the drone and publishes offboard control messages to PX4.
 * Subscribes to /folding_drone/in/actuator_commands and forwards
 * the commands to the appropriate PX4 topics.
 *
 * Converted from Python to C++ (ROS 2 / rclcpp).
 */

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <folding_drone_msgs/msg/fd_actuator_commands.hpp>

#include <cmath>
#include <array>

class OffboardPublisher : public rclcpp::Node
{
public:
    OffboardPublisher()
    : Node("offboard_publisher"),
      offboard_setpoint_counter_(0),
      arming_allowed_(false)
    {
        // ── Publishers ──────────────────────────────────────────────────
        offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

        actuator_motors_pub_ = create_publisher<px4_msgs::msg::ActuatorMotors>(
            "/fmu/in/actuator_motors", 10);

        // Not working via ActuatorServos; servo control uses VehicleCommand instead
        actuator_servos_pub_ = create_publisher<px4_msgs::msg::ActuatorServos>(
            "/fmu/in/actuator_servos", 10);

        vehicle_attitude_setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
            "/fmu/in/vehicle_attitude_setpoint", 10);

        trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        torque_setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
            "/fmu/in/vehicle_torque_setpoint", 10);

        thrust_setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
            "/fmu/in/vehicle_thrust_setpoint", 10);

        // ── Subscribers ─────────────────────────────────────────────────
        actuator_commands_sub_ = create_subscription<folding_drone_msgs::msg::FDActuatorCommands>(
            "/folding_drone/in/actuator_commands", 10,
            std::bind(&OffboardPublisher::actuator_commands_callback, this, std::placeholders::_1));

        arming_allowed_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/folding_drone/in/arming_allowed", 1,
            std::bind(&OffboardPublisher::arming_allowed_callback, this, std::placeholders::_1));
    }

private:
    // ─────────────────────── arming helpers ───────────────────────────
    void arm()
    {
        px4_msgs::msg::VehicleCommand msg;
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.param1  = 1.0f;
        msg.timestamp = timestamp_us();
        vehicle_command_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Arm Command Sent");
    }

    void disarm()
    {
        px4_msgs::msg::VehicleCommand msg;
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.param1  = 0.0f;
        msg.timestamp = timestamp_us();
        vehicle_command_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Disarm Command Sent");
    }

    // ─────────────────────── publish helpers ──────────────────────────
    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg;
        msg.position        = command_msg_.position;
        msg.velocity        = command_msg_.velocity;
        msg.acceleration    = command_msg_.acceleration;
        msg.attitude        = command_msg_.attitude;
        msg.body_rate       = command_msg_.body_rate;
        msg.thrust_and_torque = command_msg_.thrust_and_torque;
        msg.direct_actuator = command_msg_.direct_actuator;
        msg.timestamp       = timestamp_us();
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_actuator()
    {
        // Servo motor control via VehicleCommand
        px4_msgs::msg::VehicleCommand msg;
        msg.command   = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_ACTUATOR;
        msg.param1    = static_cast<float>(command_msg_.joint_angle / (M_PI / 4.0) - 1.0);
        msg.timestamp = timestamp_us();
        vehicle_command_pub_->publish(msg);
    }

    void publish_actuator_motors()
    {
        px4_msgs::msg::ActuatorMotors msg;
        msg.timestamp = timestamp_us();
        msg.control.fill(0.0f);
        // Copy up to 4 motor controls
        const auto& mc = command_msg_.motor_controls;
        for (std::size_t i = 0; i < 4 && i < mc.size(); ++i) {
            msg.control[i] = mc[i];
        }
        actuator_motors_pub_->publish(msg);
    }

    void publish_actuator_servos()
    {
        // Not currently working — kept for completeness
        px4_msgs::msg::ActuatorServos msg;
        msg.timestamp = timestamp_us();
        msg.control.fill(0.0f);
        actuator_servos_pub_->publish(msg);
    }

    void publish_vehicle_attitude()
    {
        px4_msgs::msg::VehicleAttitudeSetpoint msg;
        msg.timestamp   = timestamp_us();
        msg.q_d         = command_msg_.attitude_quaternions;
        msg.thrust_body = {0.0f, 0.0f, command_msg_.thrust_body};
        vehicle_attitude_setpoint_pub_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.timestamp = timestamp_us();
        msg.position  = command_msg_.trajectory_setpoint;
        msg.yaw       = command_msg_.yaw;
        trajectory_setpoint_pub_->publish(msg);
    }

    void publish_thrust_setpoint()
    {
        px4_msgs::msg::VehicleThrustSetpoint msg;
        msg.timestamp = timestamp_us();
        msg.xyz       = {0.0f, 0.0f, command_msg_.thrust_body};
        thrust_setpoint_pub_->publish(msg);
    }

    void publish_torque_setpoint()
    {
        px4_msgs::msg::VehicleTorqueSetpoint msg;
        msg.timestamp = timestamp_us();
        msg.xyz       = command_msg_.torque_setpoint;
        torque_setpoint_pub_->publish(msg);
    }

    void publish_vehicle_command(uint32_t command, float param1, float param2)
    {
        px4_msgs::msg::VehicleCommand msg;
        msg.command          = command;
        msg.param1           = param1;
        msg.param2           = param2;
        msg.target_system    = 1;
        msg.target_component = 1;
        msg.source_system    = 1;
        msg.source_component = 1;
        msg.from_external    = true;
        msg.timestamp        = timestamp_us();
        vehicle_command_pub_->publish(msg);
    }

    // ─────────────────────── main control dispatch ─────────────────────
    void timer_callback()
    {
        if (offboard_setpoint_counter_ == 10) {
            // Switch to offboard mode (mode 6)
            publish_vehicle_command(
                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);

            if (arming_allowed_) {
                arm();
            } else {
                // Not ready yet — reset counter and wait
                offboard_setpoint_counter_ = 0;
                RCLCPP_INFO(get_logger(), "Position Estimate not Received in last 10 timesteps");
                RCLCPP_INFO(get_logger(),
                    "If running simulation, publish \"True\" via Bool message on "
                    "/folding_drone/in/is_simulation");
                RCLCPP_INFO(get_logger(), "------------------------------------------------------------");
            }
        }

        if (offboard_setpoint_counter_ < 12) {
            ++offboard_setpoint_counter_;
        }

        publish_offboard_control_mode();

        if (command_msg_.direct_actuator) {
            publish_actuator_motors();
        } else if (command_msg_.attitude) {
            publish_vehicle_attitude();
        } else if (command_msg_.position) {
            publish_trajectory_setpoint();
        } else if (command_msg_.thrust_and_torque) {
            publish_thrust_setpoint();
            publish_torque_setpoint();
        }

        publish_actuator();
    }

    // ─────────────────────── callbacks ────────────────────────────────
    void actuator_commands_callback(
        const folding_drone_msgs::msg::FDActuatorCommands::SharedPtr msg)
    {
        command_msg_ = *msg;
        timer_callback();  // drive control from incoming message rate
    }

    void arming_allowed_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        arming_allowed_ = msg->data;
    }

    // ─────────────────────── utility ──────────────────────────────────
    uint64_t timestamp_us() const
    {
        return static_cast<uint64_t>(now().nanoseconds() / 1000);
    }

    // ─────────────────────────── members ──────────────────────────────
    int  offboard_setpoint_counter_;
    bool arming_allowed_;

    folding_drone_msgs::msg::FDActuatorCommands command_msg_;

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr      offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr           actuator_motors_pub_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr           actuator_servos_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr  vehicle_attitude_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr       trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr           vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr    torque_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr    thrust_setpoint_pub_;

    // Subscribers
    rclcpp::Subscription<folding_drone_msgs::msg::FDActuatorCommands>::SharedPtr actuator_commands_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                         arming_allowed_sub_;
};

// ─────────────────────────────── main ────────────────────────────────────────
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardPublisher>());
    rclcpp::shutdown();
    return 0;
}
