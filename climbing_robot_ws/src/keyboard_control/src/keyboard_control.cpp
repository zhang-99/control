#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

// 获取键盘输入（非阻塞方式）
int getKey() {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("keyboard_teleop");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto mode_pub = node->create_publisher<std_msgs::msg::Int32>("/mm/chassis_mode", 10);

    // 显示控制说明
    std::cout << "\n========== 键盘控制 ==========\n";
    std::cout << "  上箭头 : 前进\n";
    std::cout << "  下箭头 : 后退\n";
    std::cout << "  左箭头 : 左转\n";
    std::cout << "  右箭头 : 右转\n";
    std::cout << "  空格键 : 停止\n";
    std::cout << "  q 键   : 退出程序\n";
    std::cout << "====================================\n\n";

    geometry_msgs::msg::Twist twist_msg;
    double linear_speed = 0.5;
    double angular_speed = 0.5;
    bool running = true;

    // 使用单独的线程处理键盘输入
    std::thread input_thread([&]() {
        while (running) {
            int key = getKey();
            if (key == 'q' || key == 'Q') {
                 RCLCPP_INFO(node->get_logger(),"quit keyboard control");
                running = false;
                break;
            }

            // 清空速度消息
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;

            // 根据按键设置速度
            // 箭头键在终端中通常返回三个字符：27, 91, [A/B/C/D]
            // 这里简化处理，只捕获常见的单字符
            switch (key) {
                case 'w': // 前进
                case 'W':
                    RCLCPP_INFO(node->get_logger(),"forward");
                    twist_msg.linear.x = linear_speed;
                    break;
                case 's': // 后退
                case 'S':
                    RCLCPP_INFO(node->get_logger(),"back");
                    twist_msg.linear.x = -linear_speed;
                    break;
                case 'a': // 左转
                case 'A':
                    RCLCPP_INFO(node->get_logger(),"turn left");
                    twist_msg.angular.z = angular_speed;
                    break;
                case 'd': // 右转
                case 'D':
                    RCLCPP_INFO(node->get_logger(),"turn right");    
                    twist_msg.angular.z = -angular_speed;
                    break;
                case ' ': // 空格停止
                    RCLCPP_INFO(node->get_logger(),"stop");    
                    twist_msg.linear.x = 0.0;
                    twist_msg.angular.z = 0.0;
                    break;
                default:
                    // 不按任何方向键或无效键则保持停止
                    RCLCPP_INFO(node->get_logger(),"stop");    
                    twist_msg.linear.x = 0.0;
                    twist_msg.angular.z = 0.0;
                    break;
            }
        }
    });

    input_thread.detach();

    // 主循环：以固定频率发布速度消息
    rclcpp::Rate rate(20); // 20Hz
    while (rclcpp::ok() && running) {
        publisher->publish(twist_msg);
        std_msgs::msg::Int32 mode;
        mode.data=1;
        mode_pub->publish(mode);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // 退出前停止
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    publisher->publish(twist_msg);
     rate.sleep();
     publisher->publish(twist_msg);
    std_msgs::msg::Int32 mode;
    mode.data=0;
    mode_pub->publish(mode);
     rate.sleep();
    mode_pub->publish(mode);


    input_thread.join();
    rclcpp::shutdown();
    return 0;
}