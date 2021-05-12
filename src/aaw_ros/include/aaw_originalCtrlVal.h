#ifndef AAW_ORIGINAL_CTRL_VAL_H_
#define AAW_ORIGINAL_CTRL_VAL_H_

//仅包含并联机构的零点位置，由moveRobotServer和visualServo两个文件共享
std::vector<float> originalCtrlVal_{0, 0, 680, 0, 0, 0};
float timeIntegUsedInCoordTrans_ = 0.8; //更改时也要改"aaw_coordtransform.cpp"中的时间积分量
#endif
