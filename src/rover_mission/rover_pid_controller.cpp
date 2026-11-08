#include <memory>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class RoverPIDController : public rclcpp::Node
{
public:
    RoverPIDController() : Node("rover_pid_controller")
    {
        // Publisher ke topik cmd_vel
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Subscriber ke topik odom
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&RoverPIDController::odom_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Node PID Controller (C++) telah berjalan.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Dapatkan posisi X saat ini dari odometry
        double current_x = msg->pose.pose.position.x;
        
        // Hitung Error (dari posisi spawn -11 menuju target 0.0)
        double error = target_x_ - current_x;
        
        // Perhitungan PID
        integral_ += error;
        double derivative = error - prev_error_;
        
        double output_speed = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);
        
        prev_error_ = error;

        // Batasi (clamp) kecepatan maksimal sesuai spesifikasi model (1.0 m/s)
        output_speed = std::clamp(output_speed, -1.0, 1.0);

        auto twist = geometry_msgs::msg::Twist();

        // Toleransi jarak jika sudah sampai di pusat platform
        if (std::abs(error) < 0.1) {
            twist.linear.x = 0.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Target X tercapai! Rover berhenti.");
        } else {
            twist.linear.x = output_speed;
        }
        
        // Pastikan tidak berbelok (jika spawn sudah lurus ke +X)
        twist.angular.z = 0.0; 

        publisher_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

    // Target Misi 2 (Pusat Platform)
    double target_x_ = 0.0;

    // Parameter PID (Kamu harus men-tuning nilai ini agar kuat nanjak)
    double kp_ = 0.8;
    double ki_ = 0.01; // Tambahkan sedikit Ki agar ada tambahan torsi jika tertahan di tanjakan
    double kd_ = 0.2;
    
    double prev_error_ = 0.0;
    double integral_ = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverPIDController>());
    rclcpp::shutdown();
    return 0;
}
