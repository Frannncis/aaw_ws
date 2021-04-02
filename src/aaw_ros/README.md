# Auto Assembly Worker - ROS
Created and maintained by Zhangping Wang.

功能和操作方法如下：
## １. 校准期望位姿态：
将并联机构手动控制移动到期望位姿，固定好相机和并联机构的安装位置，在终端中输入'roslaunch aaw_ros calibrate_desired_pos.launch'，终端会输出期望位姿的数据（左视野中特征点在归一化平面上的坐标），手动将这些值更新到aawibvs.h文件中的desiredCoord_LeftView_NormalizedPlane_x(x=0 1 2 3)变量，回到工作空间执行'catkin_make'完成校准。
## ２. 启动运动控制服务端：
启动rosmaster后，在终端输入'rosrun aaw_ros aaw_moveRobotServer'，待显示Waiting for client connection...，在并联机构示教器中切换到自动模式，输入本机IP和监听端口号，连接。连接后并联机构会运动至初始位置，然后等待伺服运动控制指令。
## 3. 启动视觉伺服
在完成步骤２后，终端输入'roslaunch aaw_ros visual_servo.launch'开始视觉伺服，并联机构会从当前位姿主动找准到期望位姿。

...(待完成)