#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <geometry_msgs/msg/wrench.hpp>
#include <chrono>
#include <deque>
 
// 定义一个 ROS 2 节点类
class Read_Force : public rclcpp::Node {
public:
    Read_Force() : Node("read_force") {
        // 创建订阅者
    subscription_ = this->create_subscription<geometry_msgs::msg::Wrench>(
      "zeroed_wrench", 10,
      std::bind(&Read_Force::message_callback, this, std::placeholders::_1));
    }

    // 获取并清除所有接收到的 Wrench 数据
    std::vector<geometry_msgs::msg::Wrench> get_and_clear_received_wrenches() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<geometry_msgs::msg::Wrench> collected_data;
        while (!received_wrenches_.empty()) {
            collected_data.push_back(received_wrenches_.front());
            received_wrenches_.pop_back();
        }
        return collected_data;
    }

private:
    void message_callback(const geometry_msgs::msg::Wrench::SharedPtr msg) {
         std::lock_guard<std::mutex> lock(mutex_);
        received_wrenches_.push_back(*msg);
        
    }

    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr subscription_;
    std::deque<geometry_msgs::msg::Wrench> received_wrenches_;
    mutable std::mutex mutex_;
};

// 非 ROS 2 程序的处理函数
void process_message(const geometry_msgs::msg::Wrench &wrench) {
    // 在这里处理接收到的消息
   std::cout << "Force: (" 
              << wrench.force.x << ", " 
              << wrench.force.y << ", " 
              << wrench.force.z << ")" 
              << std::endl;
    std::cout << "Torque: (" 
              << wrench.torque.x << ", " 
              << wrench.torque.y << ", " 
              << wrench.torque.z << ")" 
              << std::endl;
}

// 获取并处理最新 Wrench 数据
void collect_and_process_latest_wrenches(const std::shared_ptr<Read_Force> &node) {
    auto collected_data = node->get_and_clear_received_wrenches();
    for (const auto &wrench : collected_data) {
        process_message(wrench);
    }
}

// 监听键盘输入的函数
void listen_for_quit_key(std::atomic<bool> &running, std::condition_variable &cv, std::mutex &mutex) {
    char input;
    while (running) {
        input = std::cin.get();
        if (input == 'q') {
            running = false;
            {
                std::lock_guard<std::mutex> lock(mutex);
                cv.notify_all();
            }
            break;
        }
    }
}
// 全局变量，用于控制主循环的运行状态
std::atomic<bool> running(true);
std::condition_variable cv;
std::mutex mutex;

int main(int argc, char *argv[]) {
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    // 创建 ROS 2 节点
    auto node = std::make_shared<Read_Force>();

    // 创建一个线程来运行 ROS 2 节点
    std::thread ros2_thread([node]() {
        rclcpp::spin(node);
    });

    // 创建一个线程来监听键盘输入
    std::thread input_thread(listen_for_quit_key, std::ref(running), std::ref(cv), std::ref(mutex));

    // 非 ROS 2 程序的主逻辑
    while (running) {
         // 收集并处理最新的 Wrench 数据
        collect_and_process_latest_wrenches(node);

        // 在这里执行非 ROS 2 程序的其他主逻辑
        // 例如，进行一些计算或控制操作
        std::cout << "Executing other main logic..." << std::endl;

        // 短暂休眠，以减少 CPU 使用率
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "Quit success ." << std::endl;

    // 清理 ROS 2
    rclcpp::shutdown();

    // 等待 ROS 2 线程退出
    ros2_thread.join();

    // 等待输入线程退出
    input_thread.join();

    return 0;
}