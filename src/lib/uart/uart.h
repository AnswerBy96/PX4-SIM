#pragma once

#include <px4_platform_common/log.h>

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <termios.h>
#include <fcntl.h>

class Uart {
private:
    int uart_fd{-1};

public:
	Uart(const char* portname , int baudrate) ;

	~Uart() {
		close(uart_fd);
	}
	bool uart_setBaud(int baudrate);

	bool send(void* message , size_t msg_len) ;

	bool receive(void* message , size_t msg_len) ;
};
