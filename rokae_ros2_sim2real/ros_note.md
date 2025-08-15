- MAKEFLAGS=-j8 colcon build #用makeflags限制编译线程
- 采样范围需要大，xyz，平面rotation

- robot_driver 发出&接收robot关节角度数据,rviz中同步动作

- ros2 topic pub /joint_command sensor_msgs/msg/JointState '{
      header:{stamp:{sec: 0, nanosec: 0}, frame_id: 'xMatePro_base'},
      name :["xmate_joint_1", "xmate_joint_2", "xmate_joint_3", "xmate_joint_4", "xmate_joint_5", "xmate_joint_6", "xmate_joint_7"],
      position: [0, 0, 0, 0, 0, 0, 0]
}'
- ros2 topic echo /joint_command
- launch_all
      - display.launch
      - rokae_controller
      - ucro
      - eas_eyehand
- ros2 launch easy_handeye2 publish.launch.py 

1. 写接收数据（JointJog）后的执行函数，验证100hz输出的关节角度能否执行
2. 验证抓取点是否准（让夹爪去抓）
- 不准则需要重新标定
3. 跑实验需要确定控制频率是否稳定（）
- 50HZ
 

