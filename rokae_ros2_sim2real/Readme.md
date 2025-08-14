## aruco_ros配置
选择带有白边的26号二维码，便于贴在5cmx5cm木块后定位二维码四个角。

## 标定功能
- cd rokae_ros2_sim2real
- source install/setup.bash
- ros2 launch easy_handeye2 my_calibrate.launch.py 

将会启动 `xMate Joint Command GUI`、 `Rviz2`两个窗口，接着在`Rviz2`的`Displays`中勾选`Image`，将会出现`easy_handeye2 rqt`窗口。
拖动GUI各关节滑块使得xMate夹取的标定块到标定位置，后按下rqt窗口的`Take Sample`记录点位，再拖动滑块至下一个点位。
直至17个点位标定后，按下`Save`，将在`.ros2/eays_eyehand2/calibrations`生成`.calib`文件。
- ros2 launch easy_handeye2 publish.launch.py 

后在Rviz2中将出现相机相对于基座的坐标系

## Q&A
1. 机械臂急停但标定未结束怎么办？
> 运行`ros launch rokae_driver rokae_controller` 再次启动控制部分，GUI此时可用。在将机械臂复位到安全位置后关闭此进程即可继续标定。

2. 