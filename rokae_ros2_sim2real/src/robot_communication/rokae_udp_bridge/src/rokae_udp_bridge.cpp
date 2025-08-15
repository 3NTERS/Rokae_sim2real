#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <chrono>

class rokaeUDPBridge : public rclcpp::Node
{
public:
    rokaeUDPBridge() : Node("rokae_udp_bridge")
    {
        // 参数
        this->declare_parameter("isaac_ip", "192.168.1.29");
        this->declare_parameter("isaac_port", 5666);
        this->declare_parameter("local_port", 5006);
        this->declare_parameter("joint_count", 16);
        
        isaac_ip_ = this->get_parameter("isaac_ip").as_string();
        isaac_port_ = this->get_parameter("isaac_port").as_int();
        local_port_ = this->get_parameter("local_port").as_int();
        joint_count_ = this->get_parameter("joint_count").as_int();
        
        // 初始化数据
        joint_positions_.resize(joint_count_, 0.0);
        joint_velocities_.resize(joint_count_, 0.0);
        target_positions_.resize(joint_count_, 0.0);
        
        // ROS2订阅和发布
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_state", 10,
            std::bind(&rokaeUDPBridge::jointStateCallback, this, std::placeholders::_1));
            
        joint_cmd_pub_ = this->create_publisher<control_msgs::msg::JointJog>("/joint_command", 10);
        
        // UDP初始化
        initUDP();
        
        // 启动线程
        running_ = true;
        send_thread_ = std::thread(&rokaeUDPBridge::sendJointStateThread, this);
        recv_thread_ = std::thread(&rokaeUDPBridge::receiveActionThread, this);
        
        RCLCPP_INFO(this->get_logger(), "rokae UDP Bridge启动完成");
    }
    
    ~rokaeUDPBridge()
    {
        running_ = false;
        if (send_thread_.joinable()) send_thread_.join();
        if (recv_thread_.joinable()) recv_thread_.join();
        close(sockfd_);
    }

private:
    void initUDP()
    {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "创建socket失败");
            return;
        }
        
        // 绑定本地端口
        struct sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;
        local_addr.sin_port = htons(local_port_);
        
        if (bind(sockfd_, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "绑定端口失败: %d", local_port_);
            return;
        }
        
        // Isaac Gym地址
        memset(&isaac_addr_, 0, sizeof(isaac_addr_));
        isaac_addr_.sin_family = AF_INET;
        isaac_addr_.sin_port = htons(isaac_port_);
        inet_pton(AF_INET, isaac_ip_.c_str(), &isaac_addr_.sin_addr);
        
        // 设置非阻塞
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1000; // 1ms超时
        setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        
        RCLCPP_INFO(this->get_logger(), "UDP初始化完成: 本地端口%d, Isaac IP: %s:%d", 
                   local_port_, isaac_ip_.c_str(), isaac_port_);
    }
    
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        
        // 更新关节状态
        if (msg->position.size() >= joint_count_) {
            for (size_t i = 0; i < joint_count_; ++i) {
                joint_positions_[i] = msg->position[i];
            }
        }
        
        if (msg->velocity.size() >= joint_count_) {
            for (size_t i = 0; i < joint_count_; ++i) {
                joint_velocities_[i] = msg->velocity[i];
            }
        }
        
        last_joint_update_ = std::chrono::steady_clock::now();
    }
    
    void sendJointStateThread()
    {
        const auto send_period = std::chrono::milliseconds(10); // 100Hz
        auto next_send = std::chrono::steady_clock::now();
        
        while (running_) {
            {
                std::lock_guard<std::mutex> lock(joint_mutex_);
                
                // 打包数据: [timestamp(8), joint_count(4), positions(4*n), velocities(4*n)]
                auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
                double timestamp = now * 1e-9;
                
                uint8_t buffer[1024];
                size_t offset = 0;
                
                // 时间戳
                memcpy(buffer + offset, &timestamp, sizeof(double));
                offset += sizeof(double);
                
                // 关节数量
                uint32_t joint_count = joint_count_;
                memcpy(buffer + offset, &joint_count, sizeof(uint32_t));
                offset += sizeof(uint32_t);
                
                // 关节位置
                for (size_t i = 0; i < joint_count_; ++i) {
                    float pos = static_cast<float>(joint_positions_[i]);
                    memcpy(buffer + offset, &pos, sizeof(float));
                    offset += sizeof(float);
                }
                
                // 关节速度
                // for (size_t i = 0; i < joint_count_; ++i) {
                //     float vel = static_cast<float>(joint_velocities_[i]);
                //     memcpy(buffer + offset, &vel, sizeof(float));
                //     offset += sizeof(float);
                // }
                
                // 发送数据
                sendto(sockfd_, buffer, offset, 0, 
                       (struct sockaddr*)&isaac_addr_, sizeof(isaac_addr_));
                // std::cout<< "SEND DATA"<<std::endl;
            }
            
            // 定时发送
            next_send += send_period;
            std::this_thread::sleep_until(next_send);
        }
    }
    
    void receiveActionThread()
    {
        uint8_t buffer[1024];
        
        while (running_) {
            // 接收Isaac Gym的action数据
            ssize_t received = recvfrom(sockfd_, buffer, sizeof(buffer), 0, nullptr, nullptr);
            
            if (received > 0) {
                parseActionData(buffer, received);
                // std::cout<< "REC DATA" <<std::endl;
            }
            // std::cout<< "NO REC DATA" <<std::endl;
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    void parseActionData(uint8_t* buffer, ssize_t size)
    {
        if (size < 12) return; // 最小包大小检查
        
        size_t offset = 0;
        
        // 解析时间戳
        double timestamp;
        memcpy(&timestamp, buffer + offset, sizeof(double));
        offset += sizeof(double);
        
        // 解析关节数量
        uint32_t joint_count;
        memcpy(&joint_count, buffer + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        
        if (joint_count != joint_count_ || size < offset + joint_count * sizeof(float)) {
            RCLCPP_WARN(this->get_logger(), "接收数据格式错误");
            return;
        }
        
        // 解析关节目标位置
        std::vector<float> actions(joint_count);
        for (size_t i = 0; i < joint_count; ++i) {
            memcpy(&actions[i], buffer + offset, sizeof(float));
            offset += sizeof(float);
            std::cout<< actions[i] <<" ";
        }
        std::cout<<std::endl;
        
        // 发布关节命令
        publishJointCommand(actions);
        
        // 计算延迟
        auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        double current_time = now * 1e-9;
        double latency = current_time - timestamp;
        
        if (latency > 0.02) { // 超过20ms警告
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "高延迟: %.1fms", latency * 1000);
        }
    }
    
    void publishJointCommand(const std::vector<float>& actions)
    {
        auto msg = control_msgs::msg::JointJog();
        msg.header.stamp = this->now();
        
        // 设置关节名称（根据实际机器人修改）
        msg.joint_names.resize(joint_count_);
        msg.displacements.resize(joint_count_);
        
        for (size_t i = 0; i < joint_count_; ++i) {
            msg.joint_names[i] = "xmate_joint_" + std::to_string(i);
            msg.displacements[i] = actions[i];
        }
        
        msg.duration = 0.01; // 10ms
        
        joint_cmd_pub_->publish(msg);
    }
    
    // 成员变量
    std::string isaac_ip_;
    int isaac_port_, local_port_, joint_count_;
    int sockfd_;
    struct sockaddr_in isaac_addr_;
    
    std::vector<double> joint_positions_, joint_velocities_, target_positions_;
    std::mutex joint_mutex_;
    std::chrono::steady_clock::time_point last_joint_update_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub_;
    
    std::thread send_thread_, recv_thread_;
    std::atomic<bool> running_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rokaeUDPBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}