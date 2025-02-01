#ifdef PLATFORM_LINUX
#include <IO/dataOuput/yg_tcp_socket.h>

#include "precompile.h"

YG_TCP_Client::YG_TCP_Client() {
    socket_fd = -1;
    socket_len = -1;
}

YG_TCP_Client::~YG_TCP_Client() {
    if (socket_fd != -1) {
        close(socket_fd);
    }
    socket_fd = -1;
    socket_len = -1;
}

bool YG_TCP_Client::InitTcpClient(const char* ip, unsigned short port,
                                  unsigned int recvTime) {
    if (inet_addr(ip) == INADDR_NONE) {
        LOGE(" ip error:%s.ip: %s ", strerror(errno), ip);
        return false;
    }
    if (socket_fd != -1) {
        close(socket_fd);
    }
    memset(&host_addr, 0, sizeof(host_addr));
    host_addr.sin_family = AF_INET;
    host_addr.sin_addr.s_addr = inet_addr(ip);
    host_addr.sin_port = htons(port);

    if ((socket_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        LOGE("Start tcp client error:%s.ip: %s port: %d", strerror(errno), ip,
             port);
        return false;
    }

    struct timeval tm = {3, 0};
    if (recvTime > 0) {
        tm.tv_sec = recvTime / 1000;
        tm.tv_usec = 1000 * (recvTime % 1000);
    }

    if (0 > setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tm,
                       sizeof(tm))) {
        LOGE("set tcp client error:%s.time:%u", strerror(errno), recvTime);
    }

    return true;
}

int YG_TCP_Client::ConnectToServer() {
    if (socket_fd < 0) {
        printf("tcp client connet to server error.socket not created.");
        return -1;
    }
    int resC =
        ::connect(socket_fd, (struct sockaddr*)&host_addr, sizeof(host_addr));

    if (resC < 0) {
        printf("connect to tcp server error:%s", strerror(errno));
        return -1;
    }

    return 1;
}

int YG_TCP_Client::closeTcpClient() {
    if (socket_fd >= 0) {
        close(socket_fd);
    }

    socket_fd = -1;
    socket_len = -1;
    return 1;
}

long YG_TCP_Client::RecvData(char* data, int length, int msg) {
    if (socket_fd < 0) {
        return -1;
    }

    int res = recv(socket_fd, data, length, msg);

    if (res < 0) {
        printf("tcp client recive error:%s.", strerror(errno));
    }
    return res;
}

long YG_TCP_Client::SendData(char* data, int length) {
    if (socket_fd < 0) {
        printf("tcp client send to server error.socket not created.%d\n",
               socket_fd);
        return -1;
    }
    int res = send(socket_fd, data, length, 0);
    return res;
}

YG_TCP_Server::YG_TCP_Server() {
    socket_server = -1;
    socket_len = -1;
}

YG_TCP_Server::~YG_TCP_Server() {
    if (socket_server != -1) {
        close(socket_server);
    }
    socket_server = -1;
    socket_len = -1;
}
bool YG_TCP_Server::InitTcpServer(unsigned short port, unsigned int size) {
    printf("Start tcp server.port: %d listen buffer:%d\n", port, size);
    if (socket_server != -1) {
        close(socket_server);
    }
    memset(&host_addr, 0, sizeof(host_addr));
    host_addr.sin_family = AF_INET;
    host_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    host_addr.sin_port = htons(port);

    if ((socket_server = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("Start tcp server error:%s.port: %d\n", strerror(errno), port);
        return false;
    }

    int res =
        bind(socket_server, (struct sockaddr*)&host_addr, sizeof(host_addr));
    if (res < 0) {
        printf("tcp server bind port error:%s.port: %d\n", strerror(errno),
               port);
        return false;
    }

    res = listen(socket_server, size);
    if (res < 0) {
        return false;
    }
    return true;
}
int YG_TCP_Server::GetAcceptSocket() {
    socklen_t addrlen = sizeof(client_addr);

    if (socket_server < 0) {
        printf("tcp server accept error.socket not created. ");
        return -1;
    }
    int res = accept(socket_server, (struct sockaddr*)&client_addr, &addrlen);
    return res;
}
void YG_TCP_Server::closeTcpClient(int socket) {
    if (socket < 0) {
        printf("tcp server accept error.socket not created. ");
    } else {
        close(socket);
    }
}
long YG_TCP_Server::RecvData(int socket, char* data, int length, int msg) {
    (void)msg;
    if (socket < 0) {
        printf("tcp server accept error.socket not created. ");
        return -1;
    }
    return recv(socket, data, length, 0);
}
long YG_TCP_Server::SendData(int socket, char* data, int length) {
    if (socket < 0) {
        printf("tcp server accept error.socket not created. ");
        return -1;
    }

    return recv(socket, data, length, 0);
}
#endif
