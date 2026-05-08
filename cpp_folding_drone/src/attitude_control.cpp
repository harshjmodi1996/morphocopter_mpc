/**
 * attitude_control.cpp
 *
 * Arms the drone and publishes offboard control messages to control
 * attitude of the UAV using PX4.
 * Publishes to /folding_drone/in/actuator_commands.
 *
 * Converted from Python to C++ (ROS 2 / rclcpp).
 * NOTE: conversion_functions (rpy2q, q2rpy, q2rotmat, convert_rotation,
 *       convert_rotation_back) must be provided as a C++ header/library.
 */

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/input_rc.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <folding_drone_msgs/msg/fd_actuator_commands.hpp>
#include <folding_drone_msgs/msg/fd_setpoint.hpp>
#include <folding_drone_msgs/msg/fd_outerloop_errors.hpp>

// User-supplied conversion utilities (equivalent to conversion_functions.py)
#include "cpp_folding_drone/conversion_functions.hpp"

#include <array>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <map>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

class AttitudeControl : public rclcpp::Node
{
public:
    AttitudeControl()
    : Node("attitude_control"),
      emergency_(false),
      arming_allowed_(false),
      joint_angle_control_established_(false),
      roll_command_(0.0),
      pitch_command_(0.0),
      yaw_command_(0.0),
      loop_i_(0),
      thrust_body_(0.0),
      average_thrust_(0.0),
      average_thrust_count_(0),
      joint_angle_(0.0),
      actual_joint_angle_(0.0),
      roll_error_(0.0),
      pitch_error_(0.0),
      yaw_error_(0.0),
      log_counter_(0),
      log_flush_interval_(120)
    {
        // Zero-initialise arrays
        integral_error_    = {0.0, 0.0, 0.0};
        position_error_    = {0.0, 0.0, 0.0};
        velocity_error_    = {0.0, 0.0, 0.0};
        acceleration_target_ = {0.0, 0.0, 0.0};
        quaternion_command_FRD_Pixhawk_ = {1.0, 0.0, 0.0, 0.0};

        last_position_timestamp_ = now().nanoseconds() * 1e-9;

        // ── PID gains & physical params — loaded from CSV, fallback to defaults ──
        g_            = 9.81;   // m/s² (constant, not tunable)
        innerloop_rate_multiplier_ = 4;
        load_gains_from_csv();

        // ── QoS ────────────────────────────────────────────────────────
        rclcpp::QoS best_effort_qos(10);
        best_effort_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        best_effort_qos.history(rclcpp::HistoryPolicy::KeepLast);

        // ── Publishers ──────────────────────────────────────────────────
        actuator_command_pub_ = create_publisher<folding_drone_msgs::msg::FDActuatorCommands>(
            "/folding_drone/in/actuator_commands", 10);
        arming_allowed_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/folding_drone/in/arming_allowed", 1);

        // ── Subscribers ─────────────────────────────────────────────────
        setpoint_sub_ = create_subscription<folding_drone_msgs::msg::FDSetpoint>(
            "/folding_drone/in/setpoint", 10,
            std::bind(&AttitudeControl::setpoint_callback, this, std::placeholders::_1));

        outerloop_errors_sub_ = create_subscription<folding_drone_msgs::msg::FDOuterloopErrors>(
            "/folding_drone/out/outerloop_errors", 10,
            std::bind(&AttitudeControl::outerloop_errors_callback, this, std::placeholders::_1));

        input_rc_sub_ = create_subscription<px4_msgs::msg::InputRc>(
            "/fmu/out/input_rc", best_effort_qos,
            std::bind(&AttitudeControl::input_rc_callback, this, std::placeholders::_1));

        actual_joint_angle_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/folding_drone/out/actual_joint_angle", best_effort_qos,
            std::bind(&AttitudeControl::actual_joint_angle_callback, this, std::placeholders::_1));

        // ── CSV log ─────────────────────────────────────────────────────
        const std::string home_path = std::string(getenv("HOME"));
        const std::string log_dir   = home_path + "/carl_ws/log/custom_logs/folding_drone/log_files/";

        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << log_dir << "attitude_controller"
            << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
        filename_ = oss.str();

        // Write header
        {
            std::ofstream hdr(filename_);
            hdr << "Time,Epx,Epy,Epz,Evx,Evy,Evz,EIx,EIy,EIz,"
                   "Roll Command(rad),Pitch Command (rad),Yaw Command (rad),"
                   "Thrust Physical (N) x,Thrust Command (scaled),Joint Angle (rad),"
                << "kpx:," << kp_diag_modified_[0]
                << ",kvx: ," << kv_diag_modified_[0]
                << ",kix: ," << ki_diag_modified_[0]
                << ",kpy: ," << kp_diag_modified_[1]
                << ",kvy: ," << kv_diag_modified_[1]
                << ",kiy: ," << ki_diag_modified_[1]
                << ",kpz: ," << kp_diag_modified_[2]
                << ",kvz: ," << kv_diag_modified_[2]
                << ",kiz: ," << ki_diag_modified_[2] << "\n";
        }

        log_file_.open(filename_, std::ios::app);
        log_file_ << std::fixed << std::setprecision(16);

        // ── Timer (120 Hz) ──────────────────────────────────────────────
        constexpr double timer_period = 1.0 / 120.0;
        timer_ = create_wall_timer(
            std::chrono::duration<double>(timer_period),
            std::bind(&AttitudeControl::timer_callback, this));
    }

    ~AttitudeControl()
    {
        if (log_file_.is_open()) {
            log_file_.flush();
            log_file_.close();
        }
    }

private:
    // ─────────────────────── gain loader ──────────────────────────────
    /**
     * Reads PID gains and physical parameters from:
     *   ~/carl_ws/config/folding_drone/gains.csv
     *
     * CSV format (header row required, order does not matter):
     *   parameter,value
     *   Kpx,1.6
     *   ...
     *
     * If the file is missing or a parameter is absent, the hardcoded
     * default is used and a warning is printed.
     */
    void load_gains_from_csv()
    {
        const std::string home_path = std::string(getenv("HOME"));
        const std::string gains_file = home_path + "/carl_ws/src/carl_ws_src/cpp_folding_drone/src/gains.csv";

        // ── Defaults ────────────────────────────────────────────────────
        std::map<std::string, double> params = {
            {"Kpx", 1.6}, {"Kpy", 1.6}, {"Kpz", 3.0},
            {"Tdx", 1.3}, {"Tdy", 1.3}, {"Tdz", 1.3},
            {"Tix", 2.0}, {"Tiy", 2.0}, {"Tiz", 2.0},
            {"mass",           1.65},
            {"tilt_limit_deg", 30.0},
            {"thrust_limit",   10000.0}
        };

        // ── Try to read file ─────────────────────────────────────────────
        std::ifstream file(gains_file);
        if (!file.is_open()) {
            RCLCPP_WARN(get_logger(),
                "Gains file not found at '%s'. Using hardcoded defaults.",
                gains_file.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Loading gains from '%s'", gains_file.c_str());
            std::string line;

            // Skip all leading comment/blank lines AND the header row
            // (works regardless of how many comment lines precede the header)
            auto trim = [](std::string& s) {
                s.erase(0, s.find_first_not_of(" \t\r\n"));
                s.erase(s.find_last_not_of(" \t\r\n") + 1);
            };
            while (std::getline(file, line)) {
                trim(line);
                if (line.empty() || line[0] == '#') continue; // comment / blank
                break; // this is the header row — consumed, stop skipping
            }

            int line_num = 1;
            while (std::getline(file, line)) {
                ++line_num;
                trim(line);
                if (line.empty() || line[0] == '#') continue; // skip blanks/comments

                std::istringstream ss(line);
                std::string key, val_str;
                if (!std::getline(ss, key, ',') || !std::getline(ss, val_str)) {
                    RCLCPP_WARN(get_logger(),
                        "Skipping malformed line %d in gains file: '%s'",
                        line_num, line.c_str());
                    continue;
                }

                trim(key); trim(val_str);

                try {
                    double val = std::stod(val_str);
                    if (params.count(key)) {
                        params[key] = val;
                        RCLCPP_INFO(get_logger(), "  %-20s = %g", key.c_str(), val);
                    } else {
                        RCLCPP_WARN(get_logger(),
                            "Unknown parameter '%s' on line %d — ignored.",
                            key.c_str(), line_num);
                    }
                } catch (const std::invalid_argument&) {
                    RCLCPP_WARN(get_logger(),
                        "Could not parse value '%s' for '%s' on line %d — using default.",
                        val_str.c_str(), key.c_str(), line_num);
                }
            }
        }

        // ── Apply parameters ─────────────────────────────────────────────
        const double Kpx = params["Kpx"], Kpy = params["Kpy"], Kpz = params["Kpz"];
        const double Tdx = params["Tdx"], Tdy = params["Tdy"], Tdz = params["Tdz"];
        const double Tix = params["Tix"], Tiy = params["Tiy"], Tiz = params["Tiz"];

        kp_diag_ = {Kpx,       Kpy,       Kpz};
        kv_diag_ = {Kpx * Tdx, Kpy * Tdy, Kpz * Tdz};
        ki_diag_ = {Kpx / Tix, Kpy / Tiy, Kpz / Tiz};

        kp_diag_modified_ = kp_diag_;
        kv_diag_modified_ = kv_diag_;
        ki_diag_modified_ = ki_diag_;

        mass_         = params["mass"];
        tilt_limit_   = params["tilt_limit_deg"] * M_PI / 180.0;
        thrust_limit_ = params["thrust_limit"];

        RCLCPP_INFO(get_logger(),
            "Gains applied — Kp:[%.3f %.3f %.3f]  Kv:[%.3f %.3f %.3f]  Ki:[%.3f %.3f %.3f]  mass:%.3f kg",
            kp_diag_[0], kp_diag_[1], kp_diag_[2],
            kv_diag_[0], kv_diag_[1], kv_diag_[2],
            ki_diag_[0], ki_diag_[1], ki_diag_[2],
            mass_);
    }

    // ─────────────────────────── timer callback ────────────────────────
    void timer_callback()
    {
        position_estimate_health_check();
        outerloop();

        folding_drone_msgs::msg::FDActuatorCommands msg;
        msg.position          = false;
        msg.velocity          = false;
        msg.acceleration      = false;
        msg.attitude          = true;
        msg.body_rate         = false;
        msg.thrust_and_torque = false;
        msg.direct_actuator   = false;
        msg.attitude_quaternions = {
            static_cast<float>(quaternion_command_FRD_Pixhawk_[0]),
            static_cast<float>(quaternion_command_FRD_Pixhawk_[1]),
            static_cast<float>(quaternion_command_FRD_Pixhawk_[2]),
            static_cast<float>(quaternion_command_FRD_Pixhawk_[3])
        };
        msg.thrust_body       = static_cast<float>(thrust_body_);
        msg.joint_angle       = static_cast<float>(actual_joint_angle_);

        actuator_command_pub_->publish(msg);
    }

    // ─────────────────────── health check ─────────────────────────────
    void position_estimate_health_check()
    {
        double now_s = now().nanoseconds() * 1e-9;
        if (!emergency_ && (now_s - last_position_timestamp_ > 3.0)) {
            RCLCPP_INFO(get_logger(),
                "========= Position Estimate not Received in last 3 s, emergency landing ==========");
            emergency_ = true;
        }
    }

    // ─────────────────────── outer loop ───────────────────────────────
    void outerloop()
    {
        timestamp_ = now().nanoseconds() * 1e-9;

        // Adapt gains based on joint angle
        if (actual_joint_angle_ > 1.20) {
            kp_diag_modified_[1] = kp_diag_[1];
            kv_diag_modified_[1] = kv_diag_[1];
        } else {
            kp_diag_modified_ = kp_diag_;
            kv_diag_modified_ = kv_diag_;
        }

        // Compute acceleration command (element-wise PID)
        std::array<double, 3> acceleration_command;
        for (int i = 0; i < 3; ++i) {
            acceleration_command[i] =
                acceleration_target_[i]
                + kp_diag_modified_[i] * position_error_[i]
                + kv_diag_modified_[i] * velocity_error_[i]
                + ki_diag_modified_[i] * integral_error_[i];
        }

        // Roll / pitch from acceleration commands
        roll_command_ = std::clamp(
            (1.0 / -g_) * (acceleration_command[0] * std::sin(yaw_command_)
                          - acceleration_command[1] * std::cos(yaw_command_)),
            -tilt_limit_, tilt_limit_);

        pitch_command_ = std::clamp(
            (1.0 / -g_) * (acceleration_command[0] * std::cos(yaw_command_)
                           + acceleration_command[1] * std::sin(yaw_command_)),
            -tilt_limit_, tilt_limit_);

        // Convert roll/pitch/yaw to quaternion (FRD-Joint frame)
        auto q_FRD_Joint = rpy2q(roll_command_, pitch_command_, yaw_command_);

        if (!emergency_) {
            quaternion_command_FRD_Pixhawk_ =
                convert_rotation_back(actual_joint_angle_, q_FRD_Joint);

            thrust_N_ = std::clamp(
                mass_ * (g_ - acceleration_command[2]), 0.0, thrust_limit_);

            if (actual_joint_angle_ < 1.20) {
                // Experiments
                thrust_body_ = std::max(
                    std::round(((-thrust_N_ + 3.75) / 39.2) * 10000.0) / 10000.0, -0.9);
            } else {
                // Compensate for propeller interaction
                thrust_body_ = std::max(
                    std::round(((-thrust_N_ + 3.75) / 39.2 * 1.0) * 10000.0) / 10000.0, -0.9);
            }

            average_thrust_ = (average_thrust_ * average_thrust_count_ + thrust_body_)
                              / (average_thrust_count_ + 1);
            ++average_thrust_count_;
        } else {
            emergency_land();
        }

        // ── Buffered CSV write ─────────────────────────────────────────
        log_file_ << timestamp_             << ","
                  << position_error_[0]    << "," << position_error_[1]    << "," << position_error_[2]    << ","
                  << velocity_error_[0]    << "," << velocity_error_[1]    << "," << velocity_error_[2]    << ","
                  << integral_error_[0]    << "," << integral_error_[1]    << "," << integral_error_[2]    << ","
                  << roll_command_         << "," << pitch_command_         << "," << yaw_command_          << ","
                  << thrust_N_             << "," << thrust_body_           << "," << actual_joint_angle_   << "\n";

        ++log_counter_;
        if (log_counter_ >= log_flush_interval_) {
            log_file_.flush();
            log_counter_ = 0;
        }
    }

    // ─────────────────────── emergency land ───────────────────────────
    void emergency_land()
    {
        RCLCPP_INFO(get_logger(), "\n\n\n\n\n EMERGENCY LANDING \n\n\n\n\n");
        quaternion_command_FRD_Pixhawk_ = {1.0, 0.0, 0.0, 0.0};
        thrust_body_ = average_thrust_ * 0.95;
    }

    // ─────────────────────── callbacks ────────────────────────────────
    void setpoint_callback(const folding_drone_msgs::msg::FDSetpoint::SharedPtr msg)
    {
        yaw_command_  = msg->yaw;
        joint_angle_  = msg->joint_angle;
    }

    void outerloop_errors_callback(
        const folding_drone_msgs::msg::FDOuterloopErrors::SharedPtr msg)
    {
        last_position_timestamp_ = now().nanoseconds() * 1e-9;

        if (joint_angle_control_established_) {
            arming_allowed_ = true;
            std_msgs::msg::Bool b;
            b.data = true;
            arming_allowed_pub_->publish(b);
        }

        position_error_[0] = msg->position_errors.x;
        position_error_[1] = msg->position_errors.y;
        position_error_[2] = msg->position_errors.z;

        velocity_error_[0] = msg->velocity_errors.x;
        velocity_error_[1] = msg->velocity_errors.y;
        velocity_error_[2] = msg->velocity_errors.z;

        integral_error_[0] = msg->integral_errors.x;
        integral_error_[1] = msg->integral_errors.y;
        integral_error_[2] = msg->integral_errors.z;

        acceleration_target_[0] = msg->acceleration_target.x;
        acceleration_target_[1] = msg->acceleration_target.y;
        acceleration_target_[2] = msg->acceleration_target.z;
    }

    void input_rc_callback(const px4_msgs::msg::InputRc::SharedPtr msg)
    {
        // RC channel 6 (index 5) > 1500 → emergency land
        emergency_ = msg->values[5] > 1500;
    }

    void actual_joint_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        joint_angle_control_established_ = true;
        actual_joint_angle_ = msg->data;
    }

    // ─────────────────────────── members ──────────────────────────────

    // State flags
    bool emergency_;
    bool arming_allowed_;
    bool joint_angle_control_established_;

    // Timing
    double last_position_timestamp_;
    double timestamp_{0.0};

    // Commands
    double roll_command_, pitch_command_, yaw_command_;
    int    loop_i_;
    double thrust_body_;
    double thrust_N_{0.0};
    std::array<double, 4> quaternion_command_FRD_Pixhawk_;

    // Averages
    double average_thrust_;
    int    average_thrust_count_;

    // Joint state
    double joint_angle_;
    double actual_joint_angle_;

    // Errors (stored as plain arrays; index 0=x, 1=y, 2=z)
    std::array<double, 3> integral_error_;
    std::array<double, 3> position_error_;
    std::array<double, 3> velocity_error_;
    std::array<double, 3> acceleration_target_;

    // Attitude errors (unused in current loops but kept for parity)
    double roll_error_, pitch_error_, yaw_error_;

    // PID gains
    std::array<double, 3> kp_diag_, kv_diag_, ki_diag_;
    std::array<double, 3> kp_diag_modified_, kv_diag_modified_, ki_diag_modified_;

    // Physical limits
    double thrust_limit_;
    double tilt_limit_;
    double mass_;
    double g_;
    int    innerloop_rate_multiplier_;

    // ROS interfaces
    rclcpp::Publisher<folding_drone_msgs::msg::FDActuatorCommands>::SharedPtr actuator_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arming_allowed_pub_;

    rclcpp::Subscription<folding_drone_msgs::msg::FDSetpoint>::SharedPtr        setpoint_sub_;
    rclcpp::Subscription<folding_drone_msgs::msg::FDOuterloopErrors>::SharedPtr outerloop_errors_sub_;
    rclcpp::Subscription<px4_msgs::msg::InputRc>::SharedPtr                     input_rc_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr                     actual_joint_angle_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Logging
    std::string   filename_;
    std::ofstream log_file_;
    int           log_counter_;
    const int     log_flush_interval_;
};

// ─────────────────────────────── main ────────────────────────────────────────
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AttitudeControl>());
    rclcpp::shutdown();
    return 0;
}
