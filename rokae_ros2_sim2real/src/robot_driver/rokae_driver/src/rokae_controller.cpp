#define  XMATEMODEL_LIB_SUPPORTED
#include <iostream>
#include <thread>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <atomic>
#include "rokae/robot.h"
#include "rokae/utility.h"
#include "rokae/planner.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>


using namespace std;
using namespace rokae;
using namespace rokae::RtSupportedFields;

class RobotController : public rclcpp::Node {
public:
    RobotController(const string& remoteIP, const string& localIP)
        : Node("robot_controller"), remoteIP_(remoteIP), localIP_(localIP), running(false), index_(0), stepCount_(0), pauseCount_(0), robot_{remoteIP, localIP} {
        //发布关节状态
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);
        //订阅关节轨迹
        trajectory_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "joint_trajectory",10,
            std::bind(&RobotController::trajectory_callback,this,std::placeholders::_1)
        );
        //订阅关节状态
        joint_command_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_command",10,
            std::bind(&RobotController::joint_command_callback,this,std::placeholders::_1)
        );
        // 在构造函数或初始化函数中
        // joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        //     "joint_states", 10, 
        //     std::bind(&YourClass::joint_state_callback, this, std::placeholders::_1));

        // joint_jog_sub_ = this->create_subscription<control_msgs::msg::JointJog>(
        //     "joint_jog", 10, 
        //     std::bind(&YourClass::joint_jog_callback, this, std::placeholders::_1));
        
        //定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), //20HZ
            std::bind(&RobotController::publish_joint_states,this)
        );
        //初始化关节名
        joint_names_ = {
            "xmate_joint_1", "xmate_joint_2", "xmate_joint_3", "xmate_joint_4", "xmate_joint_5", "xmate_joint_6", "xmate_joint_7",  
        };
        current_joint_positions_.resize(joint_names_.size(),0.0);
        target_joint_positions_.resize(joint_names_.size(),0.0);

        RCLCPP_INFO(this->get_logger(),"Robot Controller initialized");
    }
    
    void init() {
        try {
            connect();
            configureRobot();
        } catch (const std::exception &e) {
            cerr << e.what();
        }
    }
    
private:

    void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg){
        if(msg->points.empty()){
            return;
        }
        //获取目标位置，轨迹的最后一个点
        auto target_point = msg->points.back();

        if (target_point.positions.size()!=joint_names_.size()){
            RCLCPP_WARN(this->get_logger(),"incorrect numb joints");
            return;
        }
        //更新目标位置
        for(size_t i = 0;i< joint_names_.size();++i){
            target_joint_positions_[i] = target_point.positions[i];
        }

        //发送命令到实机
        send_joint_command(target_joint_positions_);
    }

    void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if(msg->position.size() != joint_names_.size()) {
            return;
        }
        // 更新目标位置（绝对位置）
        target_joint_positions_ = msg->position;
        std::cout << "get target_pos from JointState" << std::endl;
        // 发送命令到实机
        send_joint_command(target_joint_positions_);
    }

    // 新的JointJog回调函数
    void joint_jog_callback(const control_msgs::msg::JointJog::SharedPtr msg) {
        if(msg->joint_names.size() != joint_names_.size() || 
        msg->displacements.size() != joint_names_.size()) {
            return;
        }
        
        // 将位移增量添加到当前位置
        for(size_t i = 0; i < joint_names_.size(); ++i) {
            target_joint_positions_[i] += msg->displacements[i];
        }
        
        std::cout << "get target_displacement from JointJog" << std::endl;
        // 发送命令到实机
        send_joint_command(target_joint_positions_);
    }


    void send_joint_command(const std::vector<double>& positions){
        error_code ec;
        MoveAbsJCommand moveAbsJ(positions);
        // MoveAbsJCommand moveAbsJ({0.00119048, -0.0673833, -0.0132488, 1.71956, 0.00269501, 0.907211, 0.788142});//发送并执行关节角度
        robot_.moveAppend({moveAbsJ}, remoteIP_, ec);
        robot_.moveStart(ec);
        RCLCPP_INFO(this->get_logger(),"sending joint: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, ]",
        positions[0], positions[1], positions[2], positions[3], positions[4], positions[5], positions[6]
        );

        //更新位置(机械臂读取)  
        current_joint_positions_= example_basicOperation(robot_);
    }

    void publish_joint_states(){
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = this->now();
        joint_state.header.frame_id= "xMatePro7_base";

        joint_state.name =joint_names_;
        joint_state.position = example_basicOperation(robot_);

        joint_state_publisher_->publish(joint_state);
    }

    std::vector<double> example_basicOperation(rokae::xMateErProRobot& robot){ //读取关节角度
        error_code ec;
        auto temp_joint_pos = robot_.jointPos(ec); // 轴角度 [rad]
        std::cout << "" 
                << temp_joint_pos[0] << " "
                << temp_joint_pos[1] << " "
                << temp_joint_pos[2] << " "
                << temp_joint_pos[3] << " "
                << temp_joint_pos[4] << " "
                << temp_joint_pos[5] << " " 
                << temp_joint_pos[6] << "\n";    
        std::vector<double> joint_pos(7);
        for(int i=0;i<7;i++){
            joint_pos[i]= temp_joint_pos[i];
        }
        return joint_pos;

    }
   

    void connect() {
        std::cout << remoteIP_ <<std::endl;
        error_code ec;
        robot_.connectToRobot(ec);
    }

    void setpowerdown() {
        error_code ec;
        robot_.setPowerState(false, ec);
    }

    void configureRobot() { 
        error_code ec;
        robot_.setOperateMode(rokae::OperateMode::automatic, ec);
        robot_.setMotionControlMode(MotionControlMode::NrtCommand, ec);
        robot_.setPowerState(true, ec);  
    }

    

   
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> joint_names_;
    std::vector<double> current_joint_positions_;
    std::vector<double> target_joint_positions_;

    std::string remoteIP_;
    std::string localIP_;
    xMateErProRobot robot_;

    bool running; // 控制线程运行状态的标志位
    int index_;
    int stepCount_;
    int pauseCount_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    
    std::string remoteIP = "192.168.0.160";
    std::string localIP = "192.168.0.22";
    // std::error_code ec;
    // xMateErProRobot robot(remoteIP);
    // auto temp_joint_pos = robot.jointPos(ec); // 轴角度 [rad]
    //     std::cout << "" 
    //             << temp_joint_pos[0] << " "
    //             << temp_joint_pos[1] << " "
    //             << temp_joint_pos[2] << " "
    //             << temp_joint_pos[3] << " "
    //             << temp_joint_pos[4] << " "
    //             << temp_joint_pos[5] << " "
    //             << temp_joint_pos[6] << "\n";
    RobotController controller(remoteIP, localIP);
    controller.init();
    rclcpp::spin(std::make_shared<RobotController>(remoteIP, localIP)); // 确保节点持续运行
    rclcpp::shutdown();
    return 0;
}