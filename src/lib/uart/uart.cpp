#include "uart.h"

Uart::Uart(const char* portname , int baudrate)
{
	/*
		GPS1="/dev/ttyS0"
		GPS2="/dev/ttyS7"
		TEL1="/dev/ttyS6"
		TEL2="/dev/ttyS4"
		TEL3="/dev/ttyS1"
		EXT2="/dev/ttyS3"
	*/

	uart_fd = open(portname , O_RDWR | O_NOCTTY |  O_NONBLOCK);	//加上NONBLOCK，取消串口进入阻塞状态

	if(uart_fd < 0)
	{
		PX4_ERR("uart port open failed");
		return;
	}
	uart_setBaud(baudrate);
}

bool Uart::uart_setBaud(int baudrate)
{
	int speed = B9600;

	switch (baudrate)
	{
		case 9600:   speed = B9600;   break;
		case 19200:  speed = B19200;  break;
		case 38400:  speed = B38400;  break;
		case 57600:  speed = B57600;  break;
		case 115200: speed = B115200; break;
		case 230400: speed = B230400; break;
		default:
			PX4_ERR("baudrate is invalid , default 9600");
			break;
	}

	struct termios uart_config;

	/*termios 函数族提供了一个常规的终端接口，用于控制非同步通信端口。 这个结构包含了至少下列成员：*/
	/*tcflag_t c_iflag;*/      /* 输入模式 */
	/*tcflag_t c_oflag;*/      /* 输出模式 */
	/*tcflag_t c_cflag;*/      /* 控制模式 */
	/*tcflag_t c_lflag;*/      /* 本地模式 */
	/*cc_t c_cc[NCCS];*/       /* 控制字符 */

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(uart_fd, &uart_config);
	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;
	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);
	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		printf("ERR: %d (cfsetispeed)\n", termios_state);
		return false;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		printf("ERR: %d (cfsetospeed)\n", termios_state);
		return false;
	}

	if ((termios_state = tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
		printf("ERR: %d (tcsetattr)\n", termios_state);
		return false;
	}

	return true;
}

/**
 * 串口发送数据函数
 * @param 1: 要发送的数据buffer
 * @param 2: 数据buffer的大小
 * @return 成功-> ture ; 失败 -> false
 *
 */
bool Uart::send(void* message , size_t msg_len)
{
	if(write(uart_fd,message,msg_len))
		return true;
	else
		return false;
}

/**
 * 串口接收数据函数
 * @param 1: 要存放的数据buffer
 * @param 2: 数据buffer的大小
 * @return 成功-> ture ; 失败 -> false
 */
bool Uart::receive(void* message , size_t msg_len)
{
	memset(message , 0 , msg_len);
	if(read(uart_fd,message,msg_len))
		return true;
	else
		return false;
}
