#ifndef UART_H
#define UART_H

#include <iostream>
#include <fcntl.h>  // File control definitions
#include <unistd.h> // Unix standard functions
#include <termios.h> // Terminal control definitions
#include <cstring>

class UART {
public:
    UART();
    ~UART();
    bool initUART();
    bool sendDistance(int distance);

private:
    int uart_file_desctiptor;
    const char* UART_PORT = "/dev/ttyACM0";
    const int BAUD_RATE = B9600;
    const int UART_FLAGS = O_RDWR | O_NOCTTY | O_NDELAY;
    const int UART_CONFIG_FLAGS = CS8 | CREAD | CLOCAL;
};

#endif // UART_H
