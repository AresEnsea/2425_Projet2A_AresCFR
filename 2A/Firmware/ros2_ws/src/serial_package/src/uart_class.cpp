#include <iostream>
#include <fcntl.h>
#include <unistd.h>

#define SERIAL_PORT "/dev/serial10"

class Serial {
public:
    Serial(const std::string& port) {
        serialPort = open(port.c_str(), O_WRONLY | O_NOCTTY | O_NDELAY);
        if (serialPort == -1) {
            std::cerr << "Erreur : Impossible d'ouvrir le port série !" << std::endl;
        }
    }

    ~Serial() {
        if (serialPort != -1) {
            close(serialPort);
        }
    }

    void sendData(const std::string& data) {
        if (serialPort != -1) {
            write(serialPort, data.c_str(), data.size());
        } else {
            std::cerr << "Erreur : Port série non ouvert !" << std::endl;
        }
    }

private:
    int serialPort;
};

int main() {
    Serial uart(SERIAL_PORT);

    uart.sendData("Premier message\n");
    sleep(1);
    uart.sendData("Deuxième message\n");
    sleep(1);
    uart.sendData("Troisième message\n");

    return 0;
}
