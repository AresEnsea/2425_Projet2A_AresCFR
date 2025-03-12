#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>

#define SERIAL_PORT "/dev/serial10"

using namespace std::chrono_literals;

class UART_Node : public rclcpp::Node
{
public:
    UART_Node(const std::string& port)
        : Node("UART_Node"), serialPort(-1)
    {
        // Ouvrir le port série en lecture et écriture
        serialPort = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serialPort == -1) {
            std::cerr << "Erreur : Impossible d'ouvrir le port série !" << std::endl;
        } else {
            std::cout << "Port série ouvert avec succès !" << std::endl;
        }

        // Démarrer le timer pour envoyer les données périodiquement
        timer_ = this->create_wall_timer(
            500ms, std::bind(&UART_Node::timer_callback, this));
    }

    ~UART_Node() {
        if (serialPort != -1) {
            close(serialPort);
            std::cout << "Port série fermé." << std::endl;
        }
    }

private:
    int serialPort;
    rclcpp::TimerBase::SharedPtr timer_;

    void sendData(const std::string& data) {
        if (serialPort != -1) {
            ssize_t bytes_written = write(serialPort, data.c_str(), data.size());
            if (bytes_written == -1) {
                std::cerr << "Erreur lors de l'écriture sur le port série !" << std::endl;
            }
        } else {
            std::cerr << "Erreur : Port série non ouvert !" << std::endl;
        }
    }

    void timer_callback() {
        sendData("Hello World\n");
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UART_Node>(SERIAL_PORT));
    rclcpp::shutdown();
    return 0;
}
