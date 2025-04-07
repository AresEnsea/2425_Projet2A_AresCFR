#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
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
            std::chrono::milliseconds(50),  // 20 Hz, dt = 0.05 s
            std::bind(&OdometryNode::controlLoop, this));

        // Initialisation
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_theta_ = 0.0;  // Orienté vers +X
        integral_error_ = 0.0; // Intégrale de l’erreur de distance
        integral_theta_ = 0.0; // Intégrale de l’erreur angulaire
    }

private:
    double current_x_, current_y_, current_theta_;
    double target_x_, target_y_;
    double motor1_speed_, motor2_speed_;
    double integral_error_, integral_theta_; // Termes intégraux
    rclcpp::Time last_update_;
    bool first_encoder_msg_ = true;
    bool stop_motors_ = false;
    const double WHEEL_BASE = 0.239;

    // Gains PI (à ajuster selon tes besoins)
    const double Kp_linear = 10000.0;  // Gain proportionnel linéaire
    const double Ki_linear = 500.0;    // Gain intégral linéaire
    const double Kp_angular = 2000.0;  // Gain proportionnel angulaire
    const double Ki_angular = 100.0;   // Gain intégral angulaire

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
        current_theta_ = std::fmod(current_theta_ + 2 * M_PI, 2 * M_PI);
    }

    void jetsonCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        target_x_ = msg->x;
        target_y_ = msg->y;
    }

    void stopCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        stop_motors_ = msg->data;
        if (stop_motors_) {
            send_command(0.0, 0.0);
            // Réinitialiser les intégrales lors de l’arrêt
            integral_error_ = 0.0;
            integral_theta_ = 0.0;
        }
    }

    void controlLoop()
    {
        if (!jetson_sub_->get_publisher_count() || stop_motors_) {
            return;  // Ne rien faire si pas de cible ou si arrêt demandé
        }

        // Calcul des erreurs
        double error_x = target_x_ - current_x_;
        double error_y = target_y_ - current_y_;
        double error = std::sqrt(error_x * error_x + error_y * error_y);

        double target_theta = std::atan2(error_y, error_x);
        double error_theta = target_theta - current_theta_;
        error_theta = std::atan2(std::sin(error_theta), std::cos(error_theta));

        // Mise à jour des termes intégraux (dt = 0.05 s car 20 Hz)
        const double dt = 0.05;
        integral_error_ += error * dt;
        integral_theta_ += error_theta * dt;

        // Calcul des commandes PI
        /*double Kp_linear_adaptive = 82000.0 * (1.0 / (std::abs(error_theta) + 1.0));
        double v = Kp_linear_adaptive * error + Ki_linear * integral_error_;
        */
        double v = Kp_linear * error + Ki_linear * integral_error_;
        double w = Kp_angular * error_theta + Ki_angular * integral_theta_;

        // Calcul des commandes moteurs
        double cmd_motor1 = v + w * WHEEL_BASE / 2.0; // Moteur droit
        double cmd_motor2 = v - w * WHEEL_BASE / 2.0; // Moteur gauche

        // Limitation des commandes
        cmd_motor1 = std::max(-9999.0, std::min(9999.0, cmd_motor1));
        cmd_motor2 = std::max(-9999.0, std::min(9999.0, cmd_motor2));

        send_command(cmd_motor1, cmd_motor2);
    }

    void send_command(double left, double right)
    {
        auto msg = std_msgs::msg::String();
        // Pour left : si négatif, "1" + abs(left), sinon "0" + left
        std::string left_str = (left < 0) ? "1" + std::to_string(std::abs(static_cast<int>(left))) : "0" + std::to_string(static_cast<int>(left));
        // Même chose pour right
        std::string right_str = (right < 0) ? "1" + std::to_string(std::abs(static_cast<int>(right))) : "0" + std::to_string(static_cast<int>(right));
        msg.data = left_str + right_str + "000";
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
