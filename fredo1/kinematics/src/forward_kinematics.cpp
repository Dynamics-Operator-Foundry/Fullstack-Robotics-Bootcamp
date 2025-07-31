// Gen by ChatGPT
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <memory>
#include "../include/com_util.h"
#include <csignal>
#include <chrono>
#include <thread>
#include <mutex>
#include <vector>
#include <deque>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

static std::unique_ptr<com_util<fredo_msg>> 
    joint_cmd_publisher;

volatile std::sig_atomic_t stop_flag;
static fredo_msg raw_from_down;
static std::mutex sub_mutex;
static fredo_msg q_cmd;
static double alpha = 0.1;
static std::deque<fredo_msg> q_state_buffer;

void mainloop();
void get_joint_deg();
void set_joint_deg();

int main() 
{
    joint_cmd_publisher = std::make_unique<com_util<fredo_msg>>("192.168.1.255", JOINT_CMD_TOPIC, PUB);

    q_cmd.joint1 = -40;
    q_cmd.joint2 = -50;
    q_cmd.joint3 = 90;
    // threading
    std::thread lala_thread(mainloop);
    lala_thread.join();    
    
    return 0;
}

void mainloop()
{
    const double step_deg = 2.0;  // degrees per step
    const int delay_ms = 10;
    double alpha = 0.2;

    // Initialize joint angles
    q_cmd.joint1 = 0;
    q_cmd.joint2 = 0;
    q_cmd.joint3 = 0;

    fredo_msg q_cmd_prev = q_cmd;

    // Sweep joint1 from 0 to -180, then back to 0
    for (double j1 = 0; j1 >= -180.0; j1 -= step_deg) 
    {
        double target_joint1 = j1;
        q_cmd.joint1 = alpha * target_joint1 + (1.0 - alpha) * q_cmd_prev.joint1;
        q_cmd_prev.joint1 = q_cmd.joint1;

        q_cmd.time = time_util::get_time();
        joint_cmd_publisher->pub_msg(q_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }

    for (double j1 = -180.0; j1 <= 0.0; j1 += step_deg) {
        double target_joint1 = j1;
        q_cmd.joint1 = alpha * target_joint1 + (1.0 - alpha) * q_cmd_prev.joint1;
        q_cmd_prev.joint1 = q_cmd.joint1;

        q_cmd.time = time_util::get_time();
        joint_cmd_publisher->pub_msg(q_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }

    alpha = 1;

    // Sweep joint2: 0 → -90 → 90 → 0
    for (double j2 = 0; j2 >= -90.0; j2 -= step_deg) {
        double target_joint2 = j2;
        q_cmd.joint2 = alpha * target_joint2 + (1.0 - alpha) * q_cmd_prev.joint2;
        q_cmd_prev.joint2 = q_cmd.joint2;
        
        q_cmd.time = time_util::get_time();
        joint_cmd_publisher->pub_msg(q_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    for (double j2 = -90.0; j2 <= 90.0; j2 += step_deg) {
        double target_joint2 = j2;
        q_cmd.joint2 = alpha * target_joint2 + (1.0 - alpha) * q_cmd_prev.joint2;
        q_cmd_prev.joint2 = q_cmd.joint2;

        q_cmd.time = time_util::get_time();
        joint_cmd_publisher->pub_msg(q_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    for (double j2 = 90.0; j2 >= 0.0; j2 -= step_deg) {
        double target_joint2 = j2;
        q_cmd.joint2 = alpha * target_joint2 + (1.0 - alpha) * q_cmd_prev.joint2;
        q_cmd_prev.joint2 = q_cmd.joint2;

        
        q_cmd.time = time_util::get_time();
        joint_cmd_publisher->pub_msg(q_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }

    // Sweep joint3: 0 → -90 → 90 → 0
    for (double j3 = 0; j3 >= -90.0; j3 -= step_deg) {
        double target_joint3 = j3;
        q_cmd.joint3 = alpha * target_joint3 + (1.0 - alpha) * q_cmd_prev.joint3;
        q_cmd_prev.joint3 = q_cmd.joint3;

        q_cmd.time = time_util::get_time();
        joint_cmd_publisher->pub_msg(q_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    for (double j3 = -90.0; j3 <= 90.0; j3 += step_deg) {
        double target_joint3 = j3;
        q_cmd.joint3 = alpha * target_joint3 + (1.0 - alpha) * q_cmd_prev.joint3;
        q_cmd_prev.joint3 = q_cmd.joint3;

        q_cmd.time = time_util::get_time();
        joint_cmd_publisher->pub_msg(q_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    for (double j3 = 90.0; j3 >= 0.0; j3 -= step_deg) {
        double target_joint3 = j3;
        q_cmd.joint3 = alpha * target_joint3 + (1.0 - alpha) * q_cmd_prev.joint3;
        q_cmd_prev.joint3 = q_cmd.joint3;

        q_cmd.time = time_util::get_time();
        joint_cmd_publisher->pub_msg(q_cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
}
