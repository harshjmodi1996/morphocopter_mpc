/**
 * joint_angle_publisher.cpp
 *
 * Publishes PWM values to the servo motor connected to the Raspberry Pi
 * based on the joint angle received from /folding_drone/in/setpoint.
 * Also publishes the current commanded joint angle to
 * /folding_drone/out/actual_joint_angle.
 *
 * Converted from Python to C++ (ROS 2 / rclcpp).
 *
 * Dependencies:
 *   - pigpiod_if2 daemon client library  (sudo apt install libpigpio-dev)
 *   - pigpiod must be running:  sudo pigpiod
 *   - Link with: -lpigpiod_if2  (NOT -lpigpio)
 *
 * pigpiod_if2 connects to the pigpiod daemon over a socket — no root
 * privileges required, matching the behaviour of Python's pigpio.pi().
 */

#include <rclcpp/rclcpp.hpp>
#include <folding_drone_msgs/msg/fd_setpoint.hpp>
#include <std_msgs/msg/float32.hpp>

#include <pigpiod_if2.h>     // daemon client — no root required

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// GPIO pin used for servo PWM signal (BCM numbering)
static constexpr unsigned int SERVO_GPIO_PIN = 18;

class JointAnglePublisher : public rclcpp::Node
{
public:
    JointAnglePublisher()
    : Node("joint_angle_publisher"),
      rotation_speed_max_(90.0 * M_PI / 180.0), // rad/s
      angle_min_(0.0),
      angle_max_(1.54),
      pwm_min_(500),
      pwm_max_(2400),
      pi_handle_(-1),
      joint_angle_old_(0.0),
      joint_angle_desired_(0.0),
      joint_angle_desired_previous_(0.0),
      joint_angle_command_(0.0),
      setpoints_received_(0),
      change_time_(0.0),
      change_duration_(0.0),
      change_time_set_(false)
    {
        // ── Connect to pigpiod daemon ────────────────────────────────────
        // nullptr, nullptr → connect to localhost on default port 8888
        // This is equivalent to Python's pigpio.pi()
        pi_handle_ = pigpio_start(nullptr, nullptr);
        if (pi_handle_ < 0) {
            RCLCPP_FATAL(get_logger(),
                "Failed to connect to pigpiod daemon (error %d). "
                "Make sure pigpiod is running:  sudo pigpiod",
                pi_handle_);
            throw std::runtime_error("pigpiod connection failed");
        }
        RCLCPP_INFO(get_logger(),
            "Connected to pigpiod daemon (handle %d). Servo on GPIO %u",
            pi_handle_, SERVO_GPIO_PIN);

        // ── Load PWM lookup table from CSV ───────────────────────────────
        load_pwm_data();

        // ── QoS ────────────────────────────────────────────────────────
        rclcpp::QoS best_effort_qos(10);
        best_effort_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        best_effort_qos.history(rclcpp::HistoryPolicy::KeepLast);

        // ── Publishers ──────────────────────────────────────────────────
        actual_joint_angle_pub_ = create_publisher<std_msgs::msg::Float32>(
            "/folding_drone/out/actual_joint_angle", best_effort_qos);

        // ── Subscribers ─────────────────────────────────────────────────
        setpoint_sub_ = create_subscription<folding_drone_msgs::msg::FDSetpoint>(
            "/folding_drone/in/setpoint", best_effort_qos,
            std::bind(&JointAnglePublisher::setpoint_callback, this, std::placeholders::_1));

        // ── Timer (100 Hz) ───────────────────────────────────────────────
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / 100.0),
            std::bind(&JointAnglePublisher::timer_callback, this));
    }

    ~JointAnglePublisher()
    {
        if (pi_handle_ >= 0) {
            // Stop servo pulse (0 = off) then disconnect from daemon
            set_servo_pulsewidth(pi_handle_, SERVO_GPIO_PIN, 0);
            pigpio_stop(pi_handle_);
            RCLCPP_INFO(get_logger(), "Disconnected from pigpiod daemon.");
        }
    }

private:
    // ─────────────────────── PWM data loader ──────────────────────────
    /**
     * Reads ~/carl_ws/src/carl_ws_src/folding_drone/folding_drone/PWM_data.csv
     * Each row: PWM_value, joint_angle_rad
     * Populates pwm_data_ as a vector of {pwm, angle} pairs.
     * Rows that cannot be parsed as numbers are silently skipped.
     */
    void load_pwm_data()
    {
        const std::string home_path = std::string(getenv("HOME"));
        const std::string csv_path  =
            home_path + "/carl_ws/src/carl_ws_src/folding_drone/folding_drone/PWM_data.csv";

        std::ifstream file(csv_path);
        if (!file.is_open()) {
            RCLCPP_FATAL(get_logger(),
                "Cannot open PWM data file: '%s'", csv_path.c_str());
            throw std::runtime_error("PWM data file not found: " + csv_path);
        }

        std::string line;
        int row_num = 0;
        while (std::getline(file, line)) {
            ++row_num;
            if (line.empty() || line[0] == '#') continue;

            std::istringstream ss(line);
            std::string col0, col1;
            if (!std::getline(ss, col0, ',') || !std::getline(ss, col1)) {
                RCLCPP_WARN(get_logger(),
                    "Skipping malformed row %d: '%s'", row_num, line.c_str());
                continue;
            }
            try {
                double pwm   = std::stod(col0);
                double angle = std::stod(col1);
                pwm_data_.push_back({pwm, angle});
            } catch (...) {
                // Silently skip non-numeric rows (e.g. header lines)
            }
        }

        if (pwm_data_.empty()) {
            RCLCPP_FATAL(get_logger(),
                "PWM data file '%s' contained no valid entries.", csv_path.c_str());
            throw std::runtime_error("PWM data file is empty or unreadable");
        }

        RCLCPP_INFO(get_logger(),
            "Loaded %zu PWM-angle entries from '%s'",
            pwm_data_.size(), csv_path.c_str());
    }

    // ─────────────────────── PWM lookup ───────────────────────────────
    /**
     * Returns the PWM value whose stored angle is closest to target_angle.
     * Equivalent to:  PWM_data[(np.abs(PWM_data[:,1] - angle)).argmin(), 0]
     */
    double lookup_pwm(double target_angle) const
    {
        double best_pwm  = pwm_data_[0].first;
        double best_diff = std::abs(pwm_data_[0].second - target_angle);

        for (const auto& [pwm, angle] : pwm_data_) {
            double diff = std::abs(angle - target_angle);
            if (diff < best_diff) {
                best_diff = diff;
                best_pwm  = pwm;
            }
        }
        return best_pwm;
    }

    // ─────────────────────── timer callback ───────────────────────────
    void timer_callback()
    {
        if (setpoints_received_ <= 9) return; // wait for initialisation

        const double now_s = now().nanoseconds() * 1e-9;

        // Detect a new desired angle and start a timed speed-limited ramp
        if (joint_angle_desired_ != joint_angle_desired_previous_) {
            change_time_     = now_s;
            joint_angle_old_ = joint_angle_command_;
            change_duration_ = std::abs(
                (joint_angle_desired_ - joint_angle_old_) / rotation_speed_max_);
            joint_angle_desired_previous_ = joint_angle_desired_;
            change_time_set_ = true;
        }

        // Linear interpolation during the ramp, then hold at target
        if (change_time_set_ && (now_s - change_time_) <= change_duration_) {
            double t = (now_s - change_time_) / change_duration_;
            joint_angle_command_ =
                joint_angle_old_ + (joint_angle_desired_ - joint_angle_old_) * t;
        } else {
            joint_angle_command_ = joint_angle_desired_;
        }

        // Look up PWM for 4-bar mechanism and drive servo via pigpiod daemon
        unsigned int servo_pwm =
            static_cast<unsigned int>(lookup_pwm(joint_angle_command_));
        set_servo_pulsewidth(pi_handle_, SERVO_GPIO_PIN, servo_pwm);

        // Publish actual (commanded) joint angle
        std_msgs::msg::Float32 out_msg;
        out_msg.data = static_cast<float>(joint_angle_command_);
        actual_joint_angle_pub_->publish(out_msg);
    }

    // ─────────────────────── setpoint callback ────────────────────────
    void setpoint_callback(const folding_drone_msgs::msg::FDSetpoint::SharedPtr msg)
    {
        // First 10 messages: initialise starting angle from the setpoint
        // (avoids snapping from 0 to the first real command)
        if (setpoints_received_ < 10) {
            double init_angle = std::clamp(
                static_cast<double>(msg->joint_angle), angle_min_, angle_max_);
            joint_angle_desired_previous_ = init_angle;
            joint_angle_command_          = init_angle;
            ++setpoints_received_;
        }

        // Always update desired angle, clamped to hardware limits
        joint_angle_desired_ = std::clamp(
            static_cast<double>(msg->joint_angle), angle_min_, angle_max_);
    }

    // ─────────────────────────── members ──────────────────────────────

    // Configuration
    const double rotation_speed_max_; // rad/s
    const double angle_min_;          // rad
    const double angle_max_;          // rad
    const int    pwm_min_;            // µs
    const int    pwm_max_;            // µs

    // pigpiod daemon handle (returned by pigpio_start)
    int pi_handle_;

    // State
    double joint_angle_old_;
    double joint_angle_desired_;
    double joint_angle_desired_previous_;
    double joint_angle_command_;
    int    setpoints_received_;

    // Ramp tracking
    double change_time_;
    double change_duration_;
    bool   change_time_set_;

    // PWM lookup table: {pwm_value_us, joint_angle_rad}
    std::vector<std::pair<double, double>> pwm_data_;

    // ROS interfaces
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr actual_joint_angle_pub_;
    rclcpp::Subscription<folding_drone_msgs::msg::FDSetpoint>::SharedPtr setpoint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ─────────────────────────────── main ────────────────────────────────────────
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointAnglePublisher>());
    rclcpp::shutdown();
    return 0;
}
