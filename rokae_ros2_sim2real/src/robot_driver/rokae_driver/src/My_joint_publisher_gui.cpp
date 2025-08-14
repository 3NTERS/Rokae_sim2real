#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QSlider>
#include <QLabel>
#include <QTimer>
#include <QMutex>
#include <QMutexLocker>
#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <map>
#include <algorithm>
#include <functional>
#include <chrono>

// 前向声明
class JointControlWidget;

class JointCommandPublisher : public rclcpp::Node
{
public:
    JointCommandPublisher() : Node("joint_command_publisher")
    {
        // 创建发布者
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_command", 10);
        
        // 设置关节名称和初始位置
        joint_names_ = {
            "xmate_joint_1", "xmate_joint_2", "xmate_joint_3",
            "xmate_joint_4", "xmate_joint_5", "xmate_joint_6", "xmate_joint_7"
        };
        joint_positions_.resize(joint_names_.size(), 0.0);
        dragging_.resize(joint_names_.size(), false);
        frame_id_ = "xMatePro_base";

        // 订阅当前状态用于初始化滑块
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_state", 10,
            std::bind(&JointCommandPublisher::joint_state_callback, this, std::placeholders::_1));
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 添加调试信息
        RCLCPP_INFO_ONCE(this->get_logger(), "Received joint_state message with %zu joints", msg->name.size());
        
        // 第一次收到消息时，打印所有关节名称进行对比
        static bool first_time = true;
        if (first_time) {
            first_time = false;
            RCLCPP_INFO(this->get_logger(), "Expected joint names:");
            for (const auto& name : joint_names_) {
                RCLCPP_INFO(this->get_logger(), "  - %s", name.c_str());
            }
            RCLCPP_INFO(this->get_logger(), "Received joint names:");
            for (const auto& name : msg->name) {
                RCLCPP_INFO(this->get_logger(), "  - %s", name.c_str());
            }
        }
        
        std::map<int, double> updated_positions;
        
        for (size_t i = 0; i < msg->name.size(); ++i) {
            auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->name[i]);
            if (it != joint_names_.end()) {
                int idx = std::distance(joint_names_.begin(), it);
                if (!dragging_[idx]) {  // 如果滑块不在拖动状态，则更新
                    updated_positions[idx] = msg->position[i];
                    RCLCPP_DEBUG(this->get_logger(), "Updated joint %s (idx %d) to position %f", 
                                msg->name[i].c_str(), idx, msg->position[i]);
                }
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Joint %s not found in expected names", msg->name[i].c_str());
            }
        }

        {
            QMutexLocker locker(&mutex_);
            for (const auto& [idx, pos] : updated_positions) {
                joint_positions_[idx] = pos;
            }
            if (!updated_positions.empty()) {
                received_initial_state_ = true;
            }
        }

        // 通知GUI更新滑块
        if (!updated_positions.empty() && update_callback_) {
            update_callback_(updated_positions);
        }
        
        // 如果没有匹配的关节，给出警告
        if (updated_positions.empty() && !msg->name.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                "No matching joint names found in received message");
        }
    }

    void publish_all_joints()
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = frame_id_;
        msg.name = joint_names_;
        
        {
            QMutexLocker locker(&mutex_);
            msg.position = joint_positions_;
        }
        
        msg.velocity.clear();
        msg.effort.clear();

        publisher_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Published full command");
    }

    void update_joint_position(int idx, double position)
    {
        QMutexLocker locker(&mutex_);
        if (idx >= 0 && idx < static_cast<int>(joint_positions_.size())) {
            joint_positions_[idx] = position;
        }
    }

    void set_dragging(int idx, bool dragging)
    {
        if (idx >= 0 && idx < static_cast<int>(dragging_.size())) {
            dragging_[idx] = dragging;
        }
    }
    
    bool is_dragging(int idx) const
    {
        if (idx >= 0 && idx < static_cast<int>(dragging_.size())) {
            return dragging_[idx];
        }
        return false;
    }

    const std::vector<std::string>& get_joint_names() const { return joint_names_; }
    
    std::vector<double> get_joint_positions() const 
    { 
        QMutexLocker locker(&mutex_);
        return joint_positions_; 
    }
    
    // 等待接收到第一个joint_state消息
    bool wait_for_initial_state(int timeout_ms = 5000)
    {
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(timeout_ms)) {
            if (received_initial_state_) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    }

    // 设置更新回调函数（替代Qt信号槽）
    void set_update_callback(std::function<void(const std::map<int, double>&)> callback)
    {
        update_callback_ = callback;
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    
    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_;
    std::vector<bool> dragging_;
    std::string frame_id_;
    
    mutable QMutex mutex_;
    bool received_initial_state_ = false;
    
    // 替代Qt信号槽的回调函数
    std::function<void(const std::map<int, double>&)> update_callback_;
};

class JointControlWidget : public QWidget
{
public:
    JointControlWidget(std::shared_ptr<JointCommandPublisher> node, QWidget *parent = nullptr)
        : QWidget(parent), node_(node)
    {
        setWindowTitle("xMate Joint Command GUI");
        setMinimumSize(500, 400);
        
        auto layout = new QVBoxLayout(this);
        
        const auto& joint_names = node_->get_joint_names();
        
        sliders_.resize(joint_names.size());
        labels_.resize(joint_names.size());
        
        for (size_t i = 0; i < joint_names.size(); ++i) {
            // 创建标签
            labels_[i] = new QLabel(QString::fromStdString(joint_names[i]), this);
            layout->addWidget(labels_[i]);
            
            // 创建滑块
            sliders_[i] = new QSlider(Qt::Horizontal, this);
            sliders_[i]->setRange(-314, 314);  // -3.14 to 3.14, scaled by 100
            sliders_[i]->setValue(0);  // 先设置为0，稍后会更新
            sliders_[i]->setMinimumWidth(400);
            
            // 连接信号和槽（使用传统的connect方式）
            connect(sliders_[i], &QSlider::sliderPressed, [this, i]() {
                node_->set_dragging(i, true);
            });
            
            connect(sliders_[i], &QSlider::sliderReleased, [this, i]() {
                node_->set_dragging(i, false);
                node_->publish_all_joints();
            });
            
            connect(sliders_[i], &QSlider::valueChanged, [this, i](int value) {
                double position = value / 100.0;  // Convert back to radians
                node_->update_joint_position(i, position);
                // 更新标签显示当前值
                labels_[i]->setText(QString::fromStdString(node_->get_joint_names()[i]) + 
                                   QString(": %1 rad").arg(position, 0, 'f', 2));
            });
            
            layout->addWidget(sliders_[i]);
        }
        
        // 设置更新回调函数（替代Qt信号槽）
        node_->set_update_callback([this](const std::map<int, double>& positions) {
            // 使用QTimer::singleShot在主线程中执行更新
            QTimer::singleShot(0, [this, positions]() {
                update_sliders(positions);
            });
        });
        
        // 尝试等待初始状态并更新滑块
        QTimer::singleShot(100, [this]() {
            initialize_sliders_with_current_state();
        });
        
        // 添加定时器强制刷新GUI（每100ms检查一次）
        refresh_timer_ = new QTimer(this);
        connect(refresh_timer_, &QTimer::timeout, [this]() {
            force_refresh_gui();
        });
        // 添加定时器强制刷新GUI（每100ms检查一次）
        refresh_timer_ = new QTimer(this);
        connect(refresh_timer_, &QTimer::timeout, [this]() {
            force_refresh_gui();
        });
        refresh_timer_->start(100);  // 每100ms刷新一次
        
        // 添加调试定时器，每5秒显示当前状态
        debug_timer_ = new QTimer(this);
        connect(debug_timer_, &QTimer::timeout, [this]() {
            auto positions = node_->get_joint_positions();
            QString debug_msg = "Current joint positions: ";
            for (size_t i = 0; i < positions.size(); ++i) {
                debug_msg += QString("J%1:%2 ").arg(i+1).arg(positions[i], 0, 'f', 3);
            }
            RCLCPP_DEBUG(node_->get_logger(), "%s", debug_msg.toStdString().c_str());
        });
        debug_timer_->start(5000);  // 每5秒输出一次调试信息
    }

    void force_refresh_gui()
    {
        // 强制从节点获取最新的关节位置并更新GUI
        auto current_positions = node_->get_joint_positions();
        
        static int update_counter = 0;
        update_counter++;
        
        bool any_updated = false;
        for (size_t i = 0; i < current_positions.size() && i < sliders_.size(); ++i) {
            // 只有在不拖动时才更新
            if (!node_->is_dragging(i)) {
                int slider_value = static_cast<int>(current_positions[i] * 100);
                
                // 检查是否需要更新（避免不必要的更新）
                if (sliders_[i]->value() != slider_value) {
                    sliders_[i]->blockSignals(true);
                    sliders_[i]->setValue(slider_value);
                    sliders_[i]->blockSignals(false);
                    
                    // 更新标签
                    labels_[i]->setText(QString::fromStdString(node_->get_joint_names()[i]) + 
                                       QString(": %1 rad").arg(current_positions[i], 0, 'f', 2));
                    any_updated = true;
                }
            }
        }
        
        // 每50次刷新输出一次调试信息
        if (update_counter % 50 == 0) {
            RCLCPP_DEBUG(node_->get_logger(), "Force refresh #%d - Updated: %s", 
                        update_counter, any_updated ? "YES" : "NO");
        }
        
        // 强制重绘
        if (any_updated) {
            this->update();
        }
    }
    
    void update_sliders(const std::map<int, double>& positions)
    {
        for (const auto& [idx, pos] : positions) {
            if (idx >= 0 && idx < static_cast<int>(sliders_.size())) {
                // 暂时断开信号连接，避免循环更新
                sliders_[idx]->blockSignals(true);
                sliders_[idx]->setValue(static_cast<int>(pos * 100));
                sliders_[idx]->blockSignals(false);
                
                // 更新标签
                labels_[idx]->setText(QString::fromStdString(node_->get_joint_names()[idx]) + 
                                     QString(": %1 rad").arg(pos, 0, 'f', 2));
            }
        }
        
        // 强制重绘整个窗口
        this->update();
    }
    
    void initialize_sliders_with_current_state()
    {
        // 等待初始状态
        std::thread([this]() {
            if (node_->wait_for_initial_state(3000)) {  // 等待3秒
                auto positions = node_->get_joint_positions();
                // 在主线程中更新UI
                QTimer::singleShot(0, [this, positions]() {
                    for (size_t i = 0; i < positions.size(); ++i) {
                        if (i < sliders_.size()) {
                            sliders_[i]->blockSignals(true);
                            sliders_[i]->setValue(static_cast<int>(positions[i] * 100));
                            sliders_[i]->blockSignals(false);
                            
                            labels_[i]->setText(QString::fromStdString(node_->get_joint_names()[i]) + 
                                               QString(": %1 rad").arg(positions[i], 0, 'f', 2));
                        }
                    }
                    // 强制重绘
                    this->update();
                    this->repaint();  // 立即重绘
                });
            } else {
                // 如果没有收到初始状态，在终端输出警告
                QTimer::singleShot(0, [this]() {
                    RCLCPP_WARN(node_->get_logger(), 
                        "Did not receive initial joint state. Please check if /joint_state topic is being published.");
                    
                    // 即使没有初始状态，也启动强制刷新，以防后续有数据
                    RCLCPP_INFO(node_->get_logger(), "Continuing with forced GUI refresh enabled...");
                });
            }
        }).detach();
    }

private:
    std::shared_ptr<JointCommandPublisher> node_;
    std::vector<QSlider*> sliders_;
    std::vector<QLabel*> labels_;
    QTimer* refresh_timer_;  // 强制刷新定时器
    QTimer* debug_timer_;    // 调试输出定时器
};

int main(int argc, char *argv[])
{
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 初始化Qt应用程序
    QApplication app(argc, argv);
    
    // 创建ROS2节点
    auto node = std::make_shared<JointCommandPublisher>();
    
    // 在后台线程运行ROS2 spin
    std::thread ros_thread([node]() {
        rclcpp::spin(node);
    });
    
    // 给ROS节点一些时间来建立连接
    RCLCPP_INFO(node->get_logger(), "Waiting for ROS connections...");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 创建GUI
    JointControlWidget widget(node);
    widget.show();
    
    RCLCPP_INFO(node->get_logger(), "Joint Command GUI started. Listening to /joint_state topic...");
    
    // 运行Qt事件循环
    int result = app.exec();
    
    // 清理
    rclcpp::shutdown();
    if (ros_thread.joinable()) {
        ros_thread.join();
    }
    
    return result;
}