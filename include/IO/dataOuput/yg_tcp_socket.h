#ifndef YG_TCP_SOCKET_H
#define YG_TCP_SOCKET_H

#ifdef PLATFORM_LINUX
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>

class YG_TCP_Client
{
public:
    YG_TCP_Client();
    ~YG_TCP_Client();

    bool InitTcpClient(const char* ip, unsigned short port, unsigned int recvTime=0);
    int ConnectToServer();
    int closeTcpClient();
    long RecvData(char* data, int length,int msg=0);
    long SendData(char* data,int length);
    int socket_fd;

private:
    int socket_len;
    struct sockaddr_in host_addr;
};

class YG_TCP_Server
{
public:
    YG_TCP_Server();
    ~YG_TCP_Server();
    bool InitTcpServer(unsigned short port, unsigned int size);
    int GetAcceptSocket();
    void closeTcpClient(int socket);
    long RecvData(int socket,char* data, int length,int msg=0);
    long SendData(int socket,char* data,int length);

private:
    int socket_server;
    int socket_len;
    struct sockaddr_in host_addr;
    struct sockaddr_in client_addr;
};

#endif


#endif // YG_TCP_SOCKET_H
