# Auto Assembly Worker - ROS
Created and maintained by Zhangping Wang.
仔细阅读'aaw_visualServo.h'和'aaw_visualServo.cpp'中的代码和注释，即可理解本功能包的控制逻辑。
功能和操作方法如下：
## 1. 校准期望位姿态：
将并联机构手动控制移动到期望位姿（伺服终点），固定好相机和并联机构的安装位置，在终端中输入'roslaunch aaw_ros calibrate_desired_pos.launch'，终端会输出期望位姿的数据（左视野中特征点在归一化平面上的坐标），手动将这些值更新到aawibvs.h文件中的desiredCoord_LeftView_NormalizedPlane_x(x=0 1 2 3)变量，回到工作空间执行'catkin_make'
注意，当前未安装激光位移传感器的情况下，为了演示效果，还需要调整并记录对接完成状态和对准状态间在z方向上的差值。以后可以根据坐标变换矩阵实现斜着推，不一定是要z方向上的增量。
## 2. 更新零点位置（伺服起始点和结束点）
启动步骤1中的launch文件，借助OpenCV输出辅助判断特征点提取是否稳定，把并联机构手动控制调歪，将并联机构示教器中的运动控制量更新到aaw_originalCtrlVal.h文件中，回到工作空间执行'catkin_make'完成零点设置。（此步骤仅仅在当前视觉不稳定的情况下，为了演示效果才需要执行，改进视觉算法之后不需要观测特征点提取，可以任意更改该头文件中的零点位置。）
## 3. 启动电机、各传感器的服务端：
在终端输入'roslaunch aaw_ros aaw_motorAndSensors.launch'，观察终端输出可知相应的电机和传感器是否正确启动。配置该launch文件即可选择启用哪些电机和传感器，但是要注意，选择不同的电机和传感器后，需要相应地修改'aaw_visualServo.h'和'aaw_visualServo.cpp'中的内容。修改方法参考这两个文件内的注释，只需要注释或反注释部分代码，然后回工作空间重新编译即可。
## 4. 启动并联机构运动控制服务端和伺服控制主程序：
在终端输入'roslaunch aaw_ros main.launch'，终端中会输出Waiting for client connection...此时，在并联机构示教器中切换到自动模式，输入本机IP（可以设置固定IP）和监听端口号，连接（可能需要多次尝试才能连接成功）。连接后并联机构会自动使能->运动至初始位置->自动下使能，然后等待伺服运动控制指令。同时，伺服控制主程序也已经开启，手动模式下，需要新开一个终端，输入'rosservice call /restart_robot_motion_service "ToMove:True"'才可唤醒一次对接或分离动作，执行结束后会自动停止。若欲实现自动的循环动作，或者与小车通信，需要相应地修改'aaw_visualServo.h'和'aaw_visualServo.cpp'中的内容，方法参见注释。注意，与小车通信需要小车控制端启用相应的通信节点，本文件未作说明。

...(待完成)