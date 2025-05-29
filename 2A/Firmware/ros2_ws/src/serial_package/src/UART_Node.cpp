#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <iostream>
#include <fstream>
#include <string>

#define GPIO_PIN "17" // GPIO 17 (pin physique 11 sur Raspberry Pi 4)

class UART_Node : public rclcpp::Node
{
public:
    UART_Node()
        : Node("UART_Node")
    {
        // Exporter le GPIO
        std::ofstream export_file("/sys/class/gpio/export");
        if (!export_file.is_open()) {
            std::cerr << "Erreur : Impossible d'exporter le GPIO " << GPIO_PIN << std::endl;
            rclcpp::shutdown();
            throw std::runtime_error("Échec de l'exportation du GPIO");
        }
        export_file << GPIO_PIN;
        export_file.close();

        // Configurer le GPIO comme sortie
        std::ofstream direction_file("/sys/class/gpio/gpio" GPIO_PIN "/direction");
        if (!direction_file.is_open()) {
            std::cerr << "Erreur : Impossible de configurer le GPIO " << GPIO_PIN << " comme sortie" << std::endl;
            unexportGPIO();
            rclcpp::shutdown();
            throw std::runtime_error("Échec de la configuration du GPIO");
        }
        direction_file << "out";
        direction_file.close();

        std::cout << "GPIO " << GPIO_PIN << " configuré avec succès comme sortie !" << std::endl;

        // Subscriber : données de vélocité ou commandes
        subscription_msgs_to_stm = this->create_subscription<std_msgs::msg::String>(
            "msgs_to_stm", 10, std::bind(&UART_Node::msgs_callback, this, std::placeholders::_1));

        // Publisher : pour les données (peut être utilisé si vous lisez un GPIO en entrée plus tard)
        publisher_data_encoder = this->create_publisher<std_msgs::msg::String>("data_encoder", 10);

        // Subscriber pour arrêter les moteurs
        stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "stop_moteur", 10, std::bind(&UART_Node::stopCallback, this, std::placeholders::_1));

        // Publisher pour les messages STM
        stm_pub_ = this->create_publisher<std_msgs::msg::String>("msgs_to_stm", 10);
    }

    ~UART_Node() {
        // Écrire 0 sur le GPIO avant de quitter (par sécurité)
        writeGPIO("0");
        // Désexporter le GPIO
        unexportGPIO();
        std::cout << "GPIO " << GPIO_PIN << " désexporté." << std::endl;
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_msgs_to_stm;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_data_encoder;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stm_pub_;
    bool stop_motors_ = false;

    void writeGPIO(const std::string& value) {
        std::ofstream value_file("/sys/class/gpio/gpio" GPIO_PIN "/value");
        if (!value_file.is_open()) {
            std::cerr << "Erreur : Impossible d'écrire sur le GPIO " << GPIO_PIN << std::endl;
            return;
        }
        value_file << value;
        value_file.close();
        std::cout << "GPIO " << GPIO_PIN << " défini à : " << value << std::endl;
    }

    void unexportGPIO() {
        std::ofstream unexport_file("/sys/class/gpio/unexport");
        if (unexport_file.is_open()) {
            unexport_file << GPIO_PIN;
            unexport_file.close();
        }
    }

    void msgs_callback(const std_msgs::msg::String::SharedPtr msg) {
        // Accepte uniquement "1" ou "0" pour le GPIO
        if (msg->data == "1" || msg->data == "0") {
            writeGPIO(msg->data);
        } else {
            std::cerr << "Erreur : Message non valide pour GPIO, attendu '1' ou '0', reçu : " << msg->data << std::endl;
        }
    }

    void stopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (stop_motors_ != msg->data) {
            stop_motors_ = msg->data;
            auto msg_to_send = std_msgs::msg::String();
            msg_to_send.data = stop_motors_ ? "1" : "0";
            stm_pub_->publish(msg_to_send);
            writeGPIO(msg_to_send.data); // Écrire directement sur le GPIO
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UART_Node>());
    rclcpp::shutdown();
    return 0;
}
