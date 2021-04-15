# Auto Assembly Worker - ROS
Created and maintained by Zhangping Wang.

功能和操作方法如下：
## 1. 校准期望位姿态：
将并联机构手动控制移动到期望位姿，固定好相机和并联机构的安装位置，在终端中输入'roslaunch aaw_ros calibrate_desired_pos.launch'，终端会输出期望位姿的数据（左视野中特征点在归一化平面上的坐标），手动将这些值更新到aawibvs.h文件中的desiredCoord_LeftView_NormalizedPlane_x(x=0 1 2 3)变量，回到工作空间执行'catkin_make'
注意，当前未安装激光位移传感器的情况下，为了演示效果，还需要调整并记录对接完成状态和对准状态间在z方向上的差值。以后可以根据坐标变换矩阵实现斜着推，不一定是要z方向上的增量。
## 2. 更新零点位置（伺服起始点和结束点）
启动步骤1中的launch文件，借助OpenCV输出辅助判断特征点提取是否稳定，把并联机构手动控制调歪，将并联机构示教器中的运动控制量更新到aaw_originalCtrlVal.h文件中，回到工作空间执行'catkin_make'完成零点设置。（此步骤仅仅在当前视觉不稳定的情况下，为了演示效果才需要执行，改进视觉算法之后不需要观测特征点提取，可以任意更改该头文件中的零点位置。）
## 3. 启动并联机构运动控制服务端：
启动rosmaster后，在终端输入'rosrun aaw_ros aaw_moveRobotServer'，待显示Waiting for client connection...，在并联机构示教器中切换到自动模式，输入本机IP和监听端口号，连接。连接后并联机构会运动至初始位置，然后等待伺服运动控制指令。
## 4. 启动锁紧机构运动控制服务端：
给上方步进电机及其控制器上电后，确定能ping无线控制器IP后（路由器DHCP池从192.168.0.100开始依次分配），在终端输入'rosrun aaw_ros aaw_motorDriverServer_Up'，待显示The top motor is ready for drive control!即可接受运动控制。
## 5. 启动视觉伺服
在完成步骤3后，终端输入'roslaunch aaw_ros visual_servo.launch'开始视觉伺服，并联机构会从当前位姿主动找准到期望位姿，并完成对接、等待、分离、下使能机器人等动作。

...(待完成)