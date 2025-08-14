#define XMATEMODEL_LIB_SUPPORTED
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

using namespace std;
using namespace rokae;
using namespace rokae::RtSupportedFields;

class RobotController : public rclcpp::Node {
public:
    RobotController(const string& remoteIP, const string& localIP, const string& filename)
        : Node("robot_controller"), remoteIP_(remoteIP), localIP_(localIP), running(false), index_(0), stepCount_(0), pauseCount_(0), time_(0.0) {
        cmd_pub_ = this->create_publisher<std_msgs::msg::String>("allegroHand/lib_cmd", 10);
        jointAngles_ = read_joint_angles(filename);
        timeData_ = read_time_data(filename);
        jointAnglesArray_ = convert_to_arrays(jointAngles_);
    }

    struct JointData {
        double time_it;
        std::array<double, 7> joint_positions;
    };

    void init() {
        try {
            connect();
            configureRobot();
        } catch (const std::exception &e) {
            cerr << e.what();
        }
    }

    void publishCmd(const std::string &cmd) {
        auto message = std_msgs::msg::String();
        message.data = cmd;
        cmd_pub_->publish(message);
    }

    std::function<JointPosition()> createCallback() {
        return [this]() mutable {
            if (index_ < timeData_.size() - 1) {

                // if (timeData_[index_] == 0 && running == false){
                //     std::cout << "time = 0." << std::endl;
                //     ++stepCount_;
                //     for (int i = 0; i < 7; ++i) {
                //     jntPos_[i] = jointAnglesArray_[index_][i];
                //     }
                //     JointPosition cmd = {{jntPos_[0], jntPos_[1], jntPos_[2], jntPos_[3], jntPos_[4], jntPos_[5], jntPos_[6]}};
                //     time_ += 0.001;
                //     if (pauseCount_ == 1)
                //     {   
                //         if (stepCount_ >= 2000) { 
                //         stepCount_ = 0;
                //         running = true ;
                //         std::cout << "running = true." << std::endl;
                //         }
                //     }else{
                //         stepCount_ = 0;
                //         running = true ;
                //         std::cout << "running = true." << std::endl;
                //     }
                //     return cmd; // 返回当前关节位置，不更新
                // }
                // if (running = true){
                    if(pauseCount_ == 0 || pauseCount_ == 8){
                        double t = timeData_[index_] + (timeData_[index_ + 1] - timeData_[index_]) * stepCount_ / 100.0;

                        for (int i = 0; i < 7; ++i) {
                            jntPos_[i] += (jointAnglesArray_[index_ + 1][i] - jointAnglesArray_[index_][i]) / 100.0;
                        }

                        ++stepCount_;

                        JointPosition cmd = {{jntPos_[0], jntPos_[1], jntPos_[2], jntPos_[3], jntPos_[4], jntPos_[5], jntPos_[6]}};

                        if (index_ >= timeData_.size() - 2) {
                           cmd.setFinished();
                        }
                
                        time_ += 0.001;

                        if (stepCount_ >= 100) { // 每100次更新一次目标位置
                            ++index_;
                           example_basicOperation(robot_,timeData_[index_]);
                           stepCount_ = 0;
                           if(timeData_[index_] == 0){
                               // running = false;
                               ++pauseCount_;
                            }
                        }
                        return cmd;
                    }
                    else{
                        double t = timeData_[index_] + (timeData_[index_ + 1] - timeData_[index_]) * stepCount_ / 500.0;

                        for (int i = 0; i < 7; ++i) {
                            jntPos_[i] += (jointAnglesArray_[index_ + 1][i] - jointAnglesArray_[index_][i]) / 500.0;
                        }

                        ++stepCount_;

                        JointPosition cmd = {{jntPos_[0], jntPos_[1], jntPos_[2], jntPos_[3], jntPos_[4], jntPos_[5], jntPos_[6]}};

                        if (index_ >= timeData_.size() - 2) {
                           cmd.setFinished();
                        }
                
                        time_ += 0.0002;

                        if (stepCount_ >= 500) { // 每400次更新一次目标位置
                            ++index_;
                           example_basicOperation(robot_,timeData_[index_]);
                           stepCount_ = 0;
                           if(timeData_[index_] == 0){
                               // running = false;
                               ++pauseCount_;
                            }
                        }
                        return cmd;
                    }
                
                // }
            } else {
                JointPosition cmd;
                cmd.setFinished();
                return cmd;
            }
        };
    }

    void startOperation() {
        startControlLoop();
    }


    void endOperation() {
        setpowerdown();
    }

    

    void write(const std::string& filename) {
        std::ofstream file(filename);

        if (!file.is_open()) {
            std::cerr << "无法打开文件: " << filename << std::endl;
            return;
        }

        // 写入表头
        file << "Time_it";
        for (int i = 0; i < 7; ++i) {
            file << ",Joint" << i + 1;
        }
        file << "\n";

        // 写入数据
        for (const auto& data : joint_data) {
            file << data.time_it;
            for (const auto& pos : data.joint_positions) {
                file << "," << pos;
            }
            file << "\n";
        }

        file.close();
        std::cout << "数据已成功写入文件: " << filename << std::endl;
    }

private:
    void connect() {
        error_code ec;
        robot_.connectToRobot(remoteIP_, localIP_);
    }

    void setpowerdown() {
        error_code ec;
        robot_.setPowerState(false, ec);
    }

    void configureRobot() {
        error_code ec;
        robot_.setOperateMode(rokae::OperateMode::automatic, ec);
        robot_.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot_.setPowerState(true, ec);  
    }

    void example_basicOperation(rokae::xMateErProRobot &robot, double time_it){
          error_code ec;
          auto joint_pos = robot.jointPos(ec); // 轴角度 [rad]
          std::cout << "" 
                  << joint_pos[0] << " "
                  << joint_pos[1] << " "
                  << joint_pos[2] << " "
                  << joint_pos[3] << " "
                  << joint_pos[4] << " "
                  << joint_pos[5] << " "
                  << joint_pos[6] << "\n";

        // 保存当前 time_it 和关节位置
        JointData data;
        data.time_it = time_it;
        data.joint_positions = joint_pos;
        joint_data.push_back(data);

    }

    void startControlLoop() {
        std::error_code ec;
        auto rtCon = robot_.getRtMotionController().lock();

        robot_.startReceiveRobotState(std::chrono::milliseconds(1), {RtSupportedFields::jointPos_m});

        rtCon->MoveJ(0.3, robot_.jointPos(ec), jointAnglesArray_[0]);

        auto callback = createCallback();
        rtCon->setControlLoop(callback);
        jntPos_ = robot_.jointPos(ec);
        rtCon->startMove(RtControllerMode::jointPosition);
        rtCon->startLoop(true);
        std::cout << "控制结束." << std::endl;
    }

    std::vector<double> read_time_data(const std::string& filepath) {
        std::ifstream file(filepath);
        std::string line;
        std::vector<double> time_data;

        if (getline(file, line)) {
            while (getline(file, line)) {
                std::stringstream ss(line);
                std::string cell;
                int col = 0;

                while (getline(ss, cell, ',')) {
                    if (col == 0) {
                        time_data.push_back(std::stod(cell));
                    }
                    col++;
                }
            }
        }
        return time_data;
    }

    std::vector<std::vector<double>> read_joint_angles(const std::string& filepath) {
        std::ifstream file(filepath);
        std::string line;
        std::vector<std::vector<double>> joint_angles;

        if (getline(file, line)) {
            while (getline(file, line)) {
                std::stringstream ss(line);
                std::string cell;
                std::vector<double> joints;
                int col = 0;

                while (getline(ss, cell, ',')) {
                    if(col > 0 && col < 8) {
                        joints.push_back(std::stod(cell));
                    }
                    col++;
                }
                if(joints.size() == 7)
                    joint_angles.push_back(joints);
            }
        }
        return joint_angles;
    }

    std::vector<std::array<double, 7>> convert_to_arrays(const std::vector<std::vector<double>>& joint_angles) {
        std::vector<std::array<double, 7>> joint_angles_array;
        for (const auto& joints : joint_angles) {
            if (joints.size() != 7) continue;

            std::array<double, 7> arr;
            for (size_t i = 0; i < 7; ++i) {
                arr[i] = joints[i];
            }
            joint_angles_array.push_back(arr);
        }
        return joint_angles_array;
    }

    std::string remoteIP_;
    std::string localIP_;
    xMateErProRobot robot_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_; ///< ROS2 Publisher
    bool running; // 控制线程运行状态的标志位
    std::vector<JointData> joint_data;
    std::vector<std::vector<double>> jointAngles_;
    std::vector<double> timeData_;
    std::vector<std::array<double, 7>> jointAnglesArray_;
    int index_;
    int stepCount_;
    int pauseCount_;
    double time_;
    std::array<double, 7> jntPos_;
    int pauseStepCount_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::string remoteIP = "192.168.0.160";
    std::string localIP = "192.168.0.22";
    std::string filename = "/home/rokae/workspace/moveit_test/src/out/trace_0625/RRTConnect_0_trajectory.csv";
    std::string filename_1 = "/home/rokae/workspace/moveit_test/src/out/out.csv";

    RobotController controller(remoteIP, localIP, filename);
    controller.init();
    controller.startOperation();
    controller.endOperation();
    controller.write(filename_1);

    rclcpp::spin(std::make_shared<RobotController>(remoteIP, localIP, filename)); // 确保节点持续运行
    rclcpp::shutdown();
    return 0;
}
