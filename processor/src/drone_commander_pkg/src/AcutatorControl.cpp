#include "drone_commander_pkg/ActuatorControl.hpp"

UART::UART() : uart_file_desctiptor(-1) {
    initUART();
}

UART::~UART() {
    if (uart_file_desctiptor != -1) {
        close(uart_file_desctiptor);
    }
}

// Function to initialize UART
bool UART::initUART() {
    if (uart_file_desctiptor != -1) return true;

    uart_file_desctiptor = open(UART_PORT, UART_FLAGS);
    if (uart_file_desctiptor == -1) {
        std::cerr << "Failed to open UART port!\n";
        return false;
    }

    struct termios options;
    tcgetattr(uart_file_desctiptor, &options);
    cfsetispeed(&options, BAUD_RATE);
    cfsetospeed(&options, BAUD_RATE);
    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;     // 8 data bits
    options.c_cflag |= CREAD | CLOCAL;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_oflag &= ~OPOST; // Raw output
    tcsetattr(uart_file_desctiptor, TCSANOW, &options);

    return true;
}

// Function to send distance data over UART to the arduino
bool UART::sendDistance(int distance) {
    if (uart_file_desctiptor == -1 && !initUART()) {
        return false;
    }

    char msg[50];
    snprintf(msg, sizeof(msg), "Distance: %d\n", distance);

    // Send message
    int bytes_written = write(uart_file_desctiptor, msg, strlen(msg));
    if (bytes_written < 0) {
        std::cerr << "UART write failed!\n";
        return false;
    } else {
        std::cout << "Message sent: " << msg;
        return true;
    }
}