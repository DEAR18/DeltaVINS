#ifndef YG_UDP_SOCKET_H
#define YG_UDP_SOCKET_H

#include <string>

#include "arpa/inet.h"
#include "fcntl.h"
#include "netinet/in.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "sys/socket.h"
#include "sys/types.h"
#include "unistd.h"

class YG_UDP_Client {
   public:
    YG_UDP_Client();
    ~YG_UDP_Client();

    bool InitUdpClient(uint32_t ip, unsigned short port,
                       unsigned int recvTime = 0);
    bool InitUdpClient(char* ip, unsigned short port,
                       unsigned int recvTime = 0);
    long RecvData(char* data, int length);
    long SendData(char* data, int length);

   private:
    int socket_fd;
    int socket_len;
    struct sockaddr_in host_addr;
};

class YG_UDP_Server {
   public:
    YG_UDP_Server();
    ~YG_UDP_Server();

    bool InitUdpServer(unsigned short port);
    long RecvData(unsigned char* data, int length, sockaddr_in& client);
    long SendData(unsigned char* data, int length);

   private:
    int socket_fd;
    int socket_len;
    struct sockaddr_in client_addr;
};

// add UDP SOCKET
class YG_UDP_Socket {
   public:
    YG_UDP_Socket();
    ~YG_UDP_Socket();

    void SetHostAddrAndPort(std::string addr, unsigned short port);
    void BindAddrAndPort(std::string addr, unsigned short port);
    void SetReadTimeOut(unsigned int timeMS);
    long SendDataToHost(const char* data, int len);
    long SendData(const char* data, int len, uint32_t ip, unsigned short port);
    long RecvData(char* data, int len, uint32_t* ip, unsigned short* port);

    static uint32_t IPStringToUInt(std::string ipStr);
    static std::string IPUIntToString(uint32_t ip);

   private:
    int socket_fd;
    int socket_len;

    bool is_host_set;

    struct sockaddr_in host_addr;

    struct sockaddr_in send_addr;
    struct sockaddr_in recv_addr;
};

#endif  // YG_UDP_SOCKET_H
