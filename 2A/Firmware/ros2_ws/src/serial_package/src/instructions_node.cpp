#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>

class InstructionNode : public rclcpp::Node
{
public:
    InstructionNode()
        : Node("InstructionNode"), current_instruction_index_(0), stopped_(false)
    {
        // Déclarer le paramètre instruction_file
        declare_parameter<std::string>("instruction_file", "instructions");
        std::string instruction_file = get_parameter("instruction_file").as_string() + ".txt";

        // Lire les instructions depuis le fichier
        std::string file_path = ament_index_cpp::get_package_share_directory("serial_package") + "/" + instruction_file;
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Impossible d'ouvrir %s", file_path.c_str());
            throw std::runtime_error("Échec de l'ouverture du fichier d'instructions");
        }

        std::string line;
        while (std::getline(file, line)) {
            if (!line.empty()) {
                if (line == "!") {
                    break;  // Arrêter à '!'
                }
                instructions_.push_back(line);
            }
        }
        file.close();

        if (instructions_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Aucune instruction valide trouvée dans %s", file_path.c_str());
            throw std::runtime_error("Fichier d'instructions vide");
        }
        RCLCPP_INFO(this->get_logger(), "Chargé %zu instructions depuis %s", instructions_.size(), file_path.c_str());

        // Publishers et subscribers
        publisher_ = create_publisher<std_msgs::msg::String>("msgs_to_stm", 10);
        stop_moteur_sub_ = create_subscription<std_msgs::msg::Bool>(
            "stop_moteur", 10, std::bind(&InstructionNode::stop_moteur_callback, this, std::placeholders::_1));
        tirette_sub_ = create_subscription<std_msgs::msg::Bool>(
            "tirette_value", 10, std::bind(&InstructionNode::tirette_callback, this, std::placeholders::_1));

        // Timer (inactif au départ)
        timer_ = create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&InstructionNode::send_next_instruction, this));
        timer_->cancel();  // Désactiver le timer initialement
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_moteur_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tirette_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> instructions_;
    size_t current_instruction_index_;
    bool stopped_;

    void tirette_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data && current_instruction_index_ == 0 && !stopped_) {
            // Tirette désactivée (False), démarrer
            RCLCPP_INFO(this->get_logger(), "Tirette désactivée, démarrage des instructions");
            timer_->reset();  // Activer le timer
            send_next_instruction();  // Envoyer la première immédiatement
        } else if (msg->data && !stopped_) {
            // Tirette activée (True), réinitialiser
            RCLCPP_INFO(this->get_logger(), "Tirette activée, réinitialisation");
            timer_->cancel();
            current_instruction_index_ = 0;
        }
    }

    void stop_moteur_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !stopped_) {
            auto message = std_msgs::msg::String();
            message.data = ">0000000000";
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Stop moteur envoyé: %s", message.data.c_str());
            stopped_ = true;
            timer_->cancel();
        }
    }

    void send_next_instruction()
    {
        if (!stopped_ && current_instruction_index_ < instructions_.size()) {
            auto message = std_msgs::msg::String();
            message.data = instructions_[current_instruction_index_];
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Instruction envoyée: %s", message.data.c_str());
            current_instruction_index_++;
        } else if (current_instruction_index_ >= instructions_.size() && !stopped_) {
            RCLCPP_INFO(this->get_logger(), "Fin des instructions");
            stopped_ = true;
            timer_->cancel();
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<InstructionNode>());
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}
