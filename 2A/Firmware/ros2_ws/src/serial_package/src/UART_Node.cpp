#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"  // Pour le type Bool
#include "geometry_msgs/msg/pose2d.hpp"
#include <sstream>
#include <cmath>
#include <memory>

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode() : Node("odometry_node")
    {
        encoder_sub_ = this->create_subscription<std_msgs::msg::String>(
            "data_encoder", 10, std::bind(&OdometryNode::encoderCallback, this, std::placeholders::_1));
        
        jetson_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "data_jetson", 10, std::bind(&OdometryNode::jetsonCallback, this, std::placeholders::_1));
        
        stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "stop_moteur", 10, std::bind(&OdometryNode::stopCallback, this, std::placeholders::_1));

        stm_pub_ = this->create_publisher<std_msgs::msg::String>("msgs_to_stm", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20 Hz
            std::bind(&OdometryNode::controlLoop, this));

        current_x_ = 0.0;
        current_y_ = 0.0;
        current_theta_ = 0.0;  // Orienté vers +X
    }

private:
    double current_x_, current_y_, current_theta_;
    double target_x_, target_y_;
    double motor1_speed_, motor2_speed_;
    rclcpp::Time last_update_;
    bool first_encoder_msg_ = true;
    bool stop_motors_ = false;  // État d’arrêt des moteurs
    const double WHEEL_BASE = 0.239;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr encoder_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr jetson_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stm_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void encoderCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::istringstream iss(msg->data);
        iss >> motor1_speed_ >> motor2_speed_;

        rclcpp::Time current_time = now();
        if (first_encoder_msg_) {
            last_update_ = current_time;
            first_encoder_msg_ = false;
            return;
        }
        double dt = (current_time - last_update_).seconds();
        last_update_ = current_time;

        double v = (motor1_speed_ + motor2_speed_) / 2.0;
        double w = (motor1_speed_ - motor2_speed_) / WHEEL_BASE;

        current_x_ += v * cos(current_theta_) * dt;
        current_y_ += v * sin(current_theta_) * dt;
        current_theta_ += w * dt;
        current_theta_ = fmod(current_theta_ + 2 * M_PI, 2 * M_PI);
    }

    void jetsonCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        target_x_ = msg->x;
        target_y_ = msg->y;
    }

    void stopCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        stop_motors_ = msg->data;  // True (1) pour arrêter, False (0) pour continuer
        if (stop_motors_) {
            send_command(0.0, 0.0);  // Arrêt immédiat des moteurs
        }
    }

    void controlLoop()
    {
        if (!jetson_sub_->get_publisher_count() || stop_motors_) {
            return;  // Ne rien faire si pas de cible ou si arrêt demandé
        }

        const double dt = 0.05;

        double error_x = target_x_ - current_x_;
        double error_y = target_y_ - current_y_;
        double error = sqrt(error_x * error_x + error_y * error_y);

        double target_theta = atan2(error_y, error_x);
        double error_theta = target_theta - current_theta_;
        error_theta = atan2(sin(error_theta), cos(error_theta));

        double K_linear = 0.1;
        double K_angular = 0.5;
        double v = K_linear * error;
        double w = K_angular * error_theta;

        v = std::max(-1.0, std::min(1.0, v));
        w = std::max(-1.0, std::min(1.0, w));

        double cmd_motor1 = v + w * WHEEL_BASE / 2.0;
        double cmd_motor2 = v - w * WHEEL_BASE / 2.0;

        cmd_motor1 = std::max(-1.0, std::min(1.0, cmd_motor1));
        cmd_motor2 = std::max(-1.0, std::min(1.0, cmd_motor2));

        send_command(cmd_motor1, cmd_motor2);
    }

    void send_command(double left, double right)
    {
        auto msg = std_msgs::msg::String();
        msg.data = "mv " + std::to_string(left) + " " + std::to_string(right);
        stm_pub_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}
