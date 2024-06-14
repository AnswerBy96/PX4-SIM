/*
 * @Descripttion:
 * @version:
 * @Author: rsj
 * @Date: 2023-11-08 03:28:02
 * @LastEditors: rsj
 * @LastEditTime: 2024-06-05 00:06:54
 */
/**
 * @file tcp_client.cpp
 *
 */
#include "tcp.h"

TcpSocket::TcpSocket()
{
}

bool TcpSocket::InitSocket(int port)
{
	// 创建socket
	clientfd = socket(AF_INET,SOCK_STREAM,0);
	if (clientfd < 0) {
		printf("create socket failed!!\n");
		return false;
	}

	// 配置连接服务器信息
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(port);

	inet_pton(AF_INET,"192.168.0.103",&server_addr.sin_addr);

	if (connect(clientfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
	{
		close(clientfd);
		printf("Error: connect");
		return false;
    	}

	return true;
}

/**
 * 发送tcp数据
 * @param 1: 需要发送的数据
 * @param 2: 数据大小
 * @return 成功-> ture ; 失败 -> false
 */
bool TcpSocket::sendmsg(void* message , size_t msg_len) {
	// 发送数据
	if(write(clientfd,message,msg_len) < 0)
	{
		//printf("send data failed!!\n");
		return false;
	}

	return true;
}

/**
 * 接收tcp数据
 * @param 1: 存放接收到的数据结构
 * @param 2: 数据大小
 * @return 成功-> ture ; 失败 -> false
 */
bool TcpSocket::receive(void* message , size_t msg_len) {
	// 接收数据
	if (read(clientfd,message,msg_len) < 0)
	{
		//printf("Recv Data Fail");
		return false;
	}
		return true;
}
