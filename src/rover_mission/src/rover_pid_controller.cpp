
#include <memory>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

enum MissionState {
    STATE_APPROACH,
    STATE_CLIMB,
    STATE_ON_PLATFORM,
    STATE_STABILIZE,
    STATE_DONE
};

class RoverPIDController : public rclcpp::Node
{
public:
    RoverPIDController() : Node("rover_pid_controller")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&RoverPIDController::odom_callback, this, _1));

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&RoverPIDController::imu_callback, this, _1));

        RCLCPP_INFO(this->get_logger(),
            "MISI 2: ROVER PID CONTROLLER DIMULAI");
    }

private:
    double kp_ = 0.4;
    double ki_ = 0.05;
    double kd_ = 0.003;

    double integral_ = 0.0;
    double previous_error_ = 0.0;

    double current_pitch_ = 0.0;

    double target_x_ = 11.0;
    double target_y_ = 0.0;

    double normalize_angle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    double get_yaw(
        const geometry_msgs::msg::Quaternion &q)
    {
        double siny = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

        return std::atan2(siny, cosy);
    }

    double speed_pid(
        double target,
        double actual,
        double dt,
        double max_output = 0.55)
    {
        if (target <= 0.0) {
            reset_pid();
            return 0.0;
        }

        double error = target - actual;

        integral_ += error * dt;
        integral_ = std::clamp(integral_, -1.5, 1.5);

        double derivative =
            (error - previous_error_) / dt;

        double output =
            target +
            kp_ * error +
            ki_ * integral_ +
            kd_ * derivative;

        previous_error_ = error;

        return std::clamp(output, 0.0, max_output);
    }

    void reset_pid()
    {
        integral_ = 0.0;
        previous_error_ = 0.0;
    }

    void imu_callback(
        const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;

        double sinp = 2.0 * (qw * qy - qz * qx);

        if (std::abs(sinp) >= 1.0)
            current_pitch_ = std::copysign(M_PI / 2.0, sinp);
        else
            current_pitch_ = std::asin(sinp);
    }

    void odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (state_ == STATE_DONE) {
            publish_stop();
            return;
        }

        auto wall_now = std::chrono::steady_clock::now();

        double dt = std::chrono::duration<double>(
            wall_now - last_control_time_).count();

        last_control_time_ = wall_now;

        if (dt < 0.001 || dt > 0.2)
            dt = 0.02;

        double odom_x = msg->pose.pose.position.x;
        double odom_y = msg->pose.pose.position.y;

        double actual_speed =
            msg->twist.twist.linear.x;

        double yaw = get_yaw(msg->pose.pose.orientation);

        auto twist = geometry_msgs::msg::Twist();

        double heading_error =
            normalize_angle(0.0 - yaw);

        switch (state_) {

            case STATE_APPROACH:

                if (odom_x >= 0.5) {
                    state_ = STATE_CLIMB;
                    reset_pid();

                    RCLCPP_INFO(this->get_logger(),
                        "Memasuki ramp...");
                    publish_stop();
                    break;
                }

                twist.linear.x =
                    speed_pid(0.25, actual_speed, dt);

                twist.angular.z =
                    std::clamp(1.5 * heading_error, -0.45, 0.45);

                publisher_->publish(twist);

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Mendekati ramp: X = %.2f",
                    odom_x);

                break;

            case STATE_CLIMB: {

                if (std::abs(current_pitch_) > 0.08) {
                    has_climbed_ = true;
                }

                if (has_climbed_ &&
                    std::abs(current_pitch_) < 0.04 &&
                    odom_x >= 7.0)
                {
                    state_ = STATE_ON_PLATFORM;
                    reset_pid();

                    RCLCPP_INFO(this->get_logger(),
                        "Berhasil naik ke platform!");

                    publish_stop();
                    break;
                }

                twist.linear.x =
                    speed_pid(0.35, actual_speed, dt);

                twist.angular.z =
                    std::clamp(1.5 * heading_error, -0.45, 0.45);

                publisher_->publish(twist);

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Menanjak: Pitch %.3f | Speed %.2f",
                    current_pitch_,
                    actual_speed);

                break;
            }

            case STATE_ON_PLATFORM: {

                double dx = target_x_ - odom_x;
                double dy = target_y_ - odom_y;

                double distance = std::hypot(dx, dy);

                if (distance <= 0.12) {
                    state_ = STATE_STABILIZE;
                    stabilize_start_time_ = this->now();

                    stabilize_x_ = odom_x;
                    stabilize_y_ = odom_y;

                    reset_pid();
                    publish_stop();

                    RCLCPP_INFO(this->get_logger(),
                        "Tiba di pusat platform. Stabilizing...");

                    break;
                }

                double target_heading = std::atan2(dy, dx);

                heading_error =
                    normalize_angle(target_heading - yaw);

                if (std::abs(heading_error) > 0.5) {
                    twist.linear.x =
                        speed_pid(0.0, actual_speed, dt);
                } else {
                    double target_speed =
                        std::clamp(0.4 * distance, 0.08, 0.25);

                    twist.linear.x =
                        speed_pid(target_speed, actual_speed, dt, 0.30);
                }

                twist.angular.z =
                    std::clamp(2.0 * heading_error, -0.5, 0.5);

                publisher_->publish(twist);

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Menuju pusat: Jarak %.2f m",
                    distance);

                break;
            }

            case STATE_STABILIZE: {

                publish_stop();

                double distance =
                    std::hypot(target_x_ - odom_x,
                               target_y_ - odom_y);

                if (distance > 0.18) {
                    state_ = STATE_ON_PLATFORM;
                    reset_pid();

                    RCLCPP_WARN(this->get_logger(),
                        "Posisi bergeser. Mengoreksi posisi kembali...");

                    break;
                }

                double drift =
                    std::hypot(odom_x - stabilize_x_,
                               odom_y - stabilize_y_);

                if (drift > 0.05) {
                    stabilize_start_time_ = this->now();
                    stabilize_x_ = odom_x;
                    stabilize_y_ = odom_y;
                }

                double elapsed =
                    (this->now() - stabilize_start_time_).seconds();

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Stabil: %.1f / 3.0 detik",
                    elapsed);

                if (elapsed >= 3.0) {
                    state_ = STATE_DONE;

                    RCLCPP_INFO(this->get_logger(),
                        "MISI 2 BERHASIL DISELESAIKAN!");
                }

                break;
            }

            case STATE_DONE:

                publish_stop();
                break;
        }
    }

    void publish_stop()
    {
        auto twist = geometry_msgs::msg::Twist();

        twist.linear.x = 0.0;
        twist.angular.z = 0.0;

        publisher_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

    MissionState state_ = STATE_APPROACH;

    bool has_climbed_ = false;

    rclcpp::Time stabilize_start_time_;

    double stabilize_x_ = 0.0;
    double stabilize_y_ = 0.0;

    std::chrono::steady_clock::time_point last_control_time_ =
        std::chrono::steady_clock::now();
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<RoverPIDController>());

    rclcpp::shutdown();

    return 0;
}
