#include <mutex>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring> 
#include <iostream>
#include <thread>
#include "fredo.h"
#include <mutex>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <pthread.h>

#define SUB 0
#define PUB 1

namespace time_util
{
    inline double get_time()
    {
        auto now = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(now.time_since_epoch()).count();
    };

}


template<typename T>
class com_util
{
private:
    int sock;
    std::string broadcast_ip_ = "192.168.1.255";
    int broadcast_port_ = 60000;
    int listen_port_ = 60000;
    sockaddr_in broadcastAddr;
    sockaddr_in serverAddr;
    sockaddr_in clientAddr;
    socklen_t clientAddrLen;
    T buffer;
    std::mutex buffer_mutex;

public:
    // objects
    T data;
    bool sub_init = false;

    // funcs
    com_util(
        std::string broadcast_ip,
        const int broadcast_port,
        int type_of_com
    );
    void pub_msg(const T& message);

    void set_advertiser();
    void set_subscriber();
    std::pair<bool, T> callback();
    ~com_util()
    {
        close(sock);
    };
};

