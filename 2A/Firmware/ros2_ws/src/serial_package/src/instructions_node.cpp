#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <fstream>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/qos.hpp>

class InstructionNode : public rclcpp::Node
{
public:
    InstructionNode()
        : Node("InstructionNode"), stop_moteur_(false), tirette_active_(false), current_line_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "msgs_to_stm", rclcpp::QoS(10).reliable());
        subscription_stop_ = this->create_subscription<std_msgs::msg::Bool>(
            "stop_moteur", 10, std::bind(&InstructionNode::stop_moteur_callback, this, std::placeholders::_1));
        subscription_next_ = this->create_subscription<std_msgs::msg::Bool>(
            "next_command", 10, std::bind(&InstructionNode::next_command_callback, this, std::placeholders::_1));
        subscription_tirette_ = this->create_subscription<std_msgs::msg::Bool>(
            "tirette_value", 10, std::bind(&InstructionNode::tirette_callback, this, std::placeholders::_1));

        std::string package_share_directory = ament_index_cpp::get_package_share_directory("serial_package");
        instruction_file_path_ = package_share_directory + "/instructions.txt";

        file_.open(instruction_file_path_);
        if (!file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Erreur : Impossible d'ouvrir le fichier %s", instruction_file_path_.c_str());
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "InstructionNode initialisé, fichier : %s, en attente de tirette", instruction_file_path_.c_str());
    }

    ~InstructionNode() {
        if (file_.is_open()) {
            file_.close();
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_stop_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_next_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_tirette_;
    std::string instruction_file_path_;
    std::ifstream file_;
    bool stop_moteur_;
    bool tirette_active_;
    int current_line_;

    void stop_moteur_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        stop_moteur_ = msg.data;
        if (stop_moteur_) {
            RCLCPP_WARN(this->get_logger(), "Motor stop signal received. Sending stop command.");
            auto message = std_msgs::msg::String();
            message.data = ">0000000000";
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Publié stop : '%s'", message.data.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Motor resume signal received.");
        }
    }

    void tirette_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        tirette_active_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Tirette state updated: %s", tirette_active_ ? "True" : "False");
    }

    void next_command_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Reçu déclencheur sur /next_command");
            send_next_command();
        }
    }

    void send_next_command() {
        if (!tirette_active_) {
            RCLCPP_INFO(this->get_logger(), "Tirette non activée, commande ignorée.");
            return;
        }

        if (stop_moteur_) {
            RCLCPP_INFO(this->get_logger(), "Motor stopped, command ignored.");
            return;
        }

        std::string line;
        if (std::getline(file_, line)) {
            // Supprimer espaces ou caractères de fin
            line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());
            auto message = std_msgs::msg::String();
            message.data = line;
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Commande publiée : '%s' (ligne %d)", line.c_str(), current_line_);
            current_line_++;
        } else {
            RCLCPP_INFO(this->get_logger(), "Fin du fichier atteint, retour au début.");
            file_.clear();
            file_.seekg(0, std::ios::beg);
            current_line_ = 0;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InstructionNode>());
    rclcpp::shutdown();
    return 0;
}
