/*
 * @Descripttion:
 * @version:
 * @Author: rsj
 * @Date: 2023-11-08 03:28:02
 * @LastEditors: rsj
 * @LastEditTime: 2024-06-04 23:53:16
 */
/**
 * @file tcp.h
 *
 * Definition of tcp client and server.
 *
 * @author rsj
 */

#ifndef tcp_H_
#define tcp_H_

#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>

class TcpSocket {
private:
    int clientfd;
    struct sockaddr_in server_addr, client_addr;

public:
    TcpSocket();

    ~TcpSocket() {
        close(clientfd);
    }

    bool InitSocket(int port);

    bool sendmsg(void* message , size_t msg_len);

    bool receive(void* message , size_t msg_len);
};


#endif
