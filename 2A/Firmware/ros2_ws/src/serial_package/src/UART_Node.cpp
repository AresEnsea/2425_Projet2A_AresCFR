#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <atomic>

#define SERIAL_PORT "/dev/serial0"
#define BUFFER_SIZE 256

class UART_Node : public rclcpp::Node
{
public:
    UART_Node(const std::string& port)
        : Node("UART_Node"), serialPort(-1), running_(true)
    {
        // Ouvrir le port série en lecture et écriture
        serialPort = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serialPort == -1) {
            std::cerr << "Erreur : Impossible d'ouvrir le port série !" << std::endl;
            rclcpp::shutdown();
            throw std::runtime_error("Échec de l'ouverture du port série");
            
        } else {
            std::cout << "Port série ouvert avec succès !" << std::endl;
        }
        
        // subscriber : données de vélocité / ou commande de banner , take can/planck , place can/planck
        subscription_msgs_to_stm = this->create_subscription<std_msgs::msg::String>(
          "msgs_to_stm", 10, std::bind(&UART_Node::msgs_callback, this, std::placeholders::_1));
        
        // publisher : pour les données reçues de la STM
        publisher_data_encoder = this->create_publisher<std_msgs::msg::String>("data_encoder", 10);
        
        // Démarrer le thread de lecture
        read_thread_ = std::thread(&UART_Node::readSerialPort, this);
	    
    }

    ~UART_Node() {
        running_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        
        if (serialPort != -1) {
            close(serialPort);
            std::cout << "Port série fermé." << std::endl;
        }
    }
  
private:
    int serialPort;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_msgs_to_stm;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_data_encoder;
    std::thread read_thread_;
    std::atomic<bool> running_;

    void sendData(const std::string& data) {
        if (serialPort != -1) {
            ssize_t bytes_written = write(serialPort, data.c_str(), data.size());
            if (bytes_written == -1) {
                std::cerr << "Erreur lors de l'écriture sur le port série !" << std::endl;
            }else{
		std::cout << "Envoyé à la STM : " <<  data.c_str() << std::endl;
	    }
        } else {
            std::cerr << "Erreur : Port série non ouvert !" << std::endl;
        }
    }
    
    void msgs_callback(const std_msgs::msg::String::SharedPtr msg) {
	if (msg->data == "#") {
		RCLCPP_INFO(this->get_logger(), "Séparateur '#' reçu, ignoré.");
		return;
	}
        sendData(msg->data);
    }
    
    void readSerialPort() {
        char buffer[BUFFER_SIZE];
        std::string message;
        
        while (running_) {
            if (serialPort != -1) {
                ssize_t bytes_read = read(serialPort, buffer, BUFFER_SIZE - 1);
                
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0';  // Terminer la chaîne
                    
                    // Traiter les données reçues
                    for (int i = 0; i < bytes_read; i++) {
                        if (buffer[i] == '\n' || buffer[i] == '\r') {
                            // Fin du message, le publier s'il n'est pas vide
                            if (!message.empty()) {
                                auto msg = std_msgs::msg::String();
                                msg.data = message;
                                publisher_data_encoder->publish(msg);
                                std::cout << "Reçu de la STM: " << message << std::endl;
                                message.clear();
                            }
                        } else {
                            message += buffer[i];
                        }
                    }
                } else if (bytes_read < 0) {
                    std::this_thread::sleep_for(1ms);  // Éviter la surcharge CPU en cas d'erreur
                }
            } else {
                std::this_thread::sleep_for(100ms);  // Éviter la surcharge CPU si le port est fermé
            }
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UART_Node>(SERIAL_PORT));
    rclcpp::shutdown();
    return 0;
}
