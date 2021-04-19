/*本程序实现从keyboard_control_pub.py订阅move.msg并解算，生成CAN信号并发送*/

#include <ros/ros.h>
#include <ECanVci.h>
#include <canbus/MoveCar.h>
#include <canbus/AdjustCarPos.h>
#include <canbus/RestartRobotMotion.h>

#define MAX_CHANNELS  2
#define CHECK_POINT  200

unsigned int gDevType = 3;       //设备类型，USBCAN1-PRO：3
unsigned int gDevIdx = 0;        //设备索引号，默认0
unsigned int gChMask = 1;        //打开第几路CAN，打开CAN1：1，打开CAN2：2，同时打开CAN1，CAN2：3
unsigned int gBaud = 0x1c00;     //波特率，1000k:0x1400;500k:0x1c00
unsigned char gTxType = 0;       //工作模式，默认0

//发送控制报文--------------------------------------
int watchDog = 0;       //心跳信号
int watchDog2 = 0;      //二进制左移4位的心跳信号
unsigned int gearEnable = 0;     //档位使能 0-disable 1-enable
unsigned int gear = 3;           //档位 02-后退 03-空档 04-前进
unsigned int steeringEnable = 0; //转向使能 0-disable 1-enable
float steering = 0;              //转角 0.043945度/bit offset-90
unsigned int driveEnable = 0;    //驱动使能 0-disable 1-enable
float driveSpeed = 0;            //目标车速 0.04m/s/bit   max10.24m/s
unsigned int brakingEnable = 0;  //制动使能 0-disable 1-enable
float braking = 0;               //制动踏板开度  0.390625%/bit   0-100
unsigned int parkingEnable = 0;  //驻车使能 0-disable 1-enable
unsigned int parking = 1;        //驻车请求 0-release 1-apply
unsigned int odometerEnable = 1; //里程计使能 0-disable 1-enable
unsigned int odometerSetZero = 0;//里程计清零 0-disable 1-enable
unsigned int headLight = 0;      //大灯 0-disable 1-enable
unsigned int turnLightLeft = 0;  //左转向灯 0-disable 1-enable
unsigned int turnLightRight = 0; //右转向灯 0-disable 1-enable
unsigned int speaker = 0;        //蜂鸣器 0-disable 1-enable

//接收反馈报文
CAN_OBJ canRxDriveFeedback1,canRxDriveFeedback2,canRxWheelSpdFeedback1,canRxWheelSpdFeedback2,can_rx;
float speedFeedback = 0;        //
float odometer = 0;             //lichengji
float wheelSpdFeedback_RL = 0;  //左后轮轮速，0.04m/s/bit，偏移量-8
float wheelSpdFeedback_RR = 0;  //右后轮轮速，0.04m/s/bit，偏移量-8
//心跳，校验
unsigned int watchDog_Feedback = 0;     //心跳信号
unsigned int check_Feedback = 0;        //校验
float odm1 = 0;
float odm2 = 0;
float odm3 = 0;
double sec1 = 0;
double sec2 = 0;
unsigned int cycle = 0;
float targetspeed = 0.2;
float forwardtime = 10;
float backtime = 10;
float creeptime = 1;
float creepspeed = 0.15;
ros::Timer timer1;
ros::Timer timer2;
ros::Timer timer4Creeping;

ros::ServiceClient * restartRobotClientPtr;

// typedef struct _CAN_OBJ {
// UINT ID;帧ID
// UINT TimeStamp;接收到信息帧时的时间标识,从CAN控制器初始化开始计时,单位微秒。
// BYTE TimeFlag;是否使用时间标识,为1时TimeStamp有效,TimeFlag和TimeStamp只在此帧为接收帧时有意义。
// BYTE SendType;发送帧类型。=0时为正常发送,=1时为单次发送(不自动重发),=2时为自发自收(用于测试CAN卡是否损坏),=3时为单次自发自收(只发送一次,用于自测试),只在此帧为发送帧时有意义。
// BYTE RemoteFlag;是否是远程帧。=0时为数据帧,=1时为远程帧。
// BYTE ExternFlag;是否是扩展帧。=0时为标准帧(11位帧ID),=1时为扩展帧(29位帧ID)。
// BYTE DataLen;数据长度DLC(<=8),即Data的长度。
// BYTE Data[8];CAN报文的数据。空间受DataLen的约束。
// BYTE Reserved[3];系统保留。
// } CAN_OBJ, *P_CAN_OBJ;
void moveforward();
void movebackward();
void creep(unsigned int geararg);
int GearCmd(unsigned int gearEnable,unsigned int gear,unsigned int watchDog ,CAN_OBJ *can){
    memset(can, 0, sizeof(CAN_OBJ));    //全部置0
    can->ID = 0x18c4d1d0;
    can->SendType = 0 & 0xff;
    can->RemoteFlag = 0 & 0xff;
    can->ExternFlag = 1 & 0xff;
    can->DataLen = 8 & 0xff;
    can->Data[0] = gearEnable & 0xff;
    can->Data[1] = gear & 0xff;
    can->Data[6] = watchDog & 0xff;
    can->Data[7] = can->Data[0]^can->Data[1]^can->Data[2]^can->Data[3]^can->Data[4]^can->Data[5]^can->Data[6];
    Transmit(gDevType, gDevIdx, 0, can, 1);
    return 1;
}

int SteeringCmd(unsigned int steeringEnable,unsigned int steering,unsigned int watchDog ,CAN_OBJ *can){
    memset(can, 0, sizeof(CAN_OBJ));    //全部置0
    can->ID = 0x18c4d2d0;
    can->SendType = 0 & 0xff;
    can->RemoteFlag = 0 & 0xff;
    can->ExternFlag = 1 & 0xff;
    can->DataLen = 8 & 0xff;
    can->Data[0] = steeringEnable & 0xff;
    can->Data[1] = (unsigned int)((steering+90)/0.043945) & 0xff;          //转向低位,00-ff      0-4095对应-90度到90度
    can->Data[2] = ((unsigned int)((steering+90)/0.043945) >> 8) & 0xff;    //转向高位,00-0f      转向限位正负24度，1501-2594
    can->Data[6] = watchDog & 0xff;
    can->Data[7] = can->Data[0]^can->Data[1]^can->Data[2]^can->Data[3]^can->Data[4]^can->Data[5]^can->Data[6];
    Transmit(gDevType, gDevIdx, 0, can, 1);
    return 1;
}

int DriveCmd(unsigned int driveEnable,float driveSpeed,unsigned int watchDog ,CAN_OBJ *can){
    memset(can, 0, sizeof(CAN_OBJ));    //全部置0
    can->ID = 0x18c4d3d0;
    can->SendType = 0 & 0xff;
    can->RemoteFlag = 0 & 0xff;
    can->ExternFlag = 1 & 0xff;
    can->DataLen = 8 & 0xff;
    can->Data[0] = driveEnable & 0xff;
    can->Data[1] = (unsigned int)(driveSpeed/0.04) & 0xff;     //0.04m/s/bit   max256*0.04=10.24m/s
    can->Data[6] = watchDog & 0xff;
    can->Data[7] = can->Data[0]^can->Data[1]^can->Data[2]^can->Data[3]^can->Data[4]^can->Data[5]^can->Data[6];
    Transmit(gDevType, gDevIdx, 0, can, 1);
    return 1;
}

int BrakingCmd(unsigned int brakingEnable,unsigned int braking,unsigned int watchDog ,CAN_OBJ *can){
    memset(can, 0, sizeof(CAN_OBJ));    //全部置0
    can->ID = 0x18c4d4d0;
    can->SendType = 0 & 0xff;
    can->RemoteFlag = 0 & 0xff;
    can->ExternFlag = 1 & 0xff;
    can->DataLen = 8 & 0xff;
    can->Data[0] = brakingEnable & 0xff;
    can->Data[1] = (unsigned int)(braking/0.390625) & 0xff;
    can->Data[6] = watchDog & 0xff;
    can->Data[7] = can->Data[0]^can->Data[1]^can->Data[2]^can->Data[3]^can->Data[4]^can->Data[5]^can->Data[6];
    Transmit(gDevType, gDevIdx, 0, can, 1);
    return 1;
}

int ParkingCmd(unsigned int parkingEnable,unsigned int parking,unsigned int watchDog ,CAN_OBJ *can){
    memset(can, 0, sizeof(CAN_OBJ));    //全部置0
    can->ID = 0x18c4d5d0;
    can->SendType = 0 & 0xff;
    can->RemoteFlag = 0 & 0xff;
    can->ExternFlag = 1 & 0xff;
    can->DataLen = 8 & 0xff;
    can->Data[0] = parkingEnable & 0xff;
    can->Data[1] = parking & 0xff;
    can->Data[6] = watchDog & 0xff;
    can->Data[7] = can->Data[0]^can->Data[1]^can->Data[2]^can->Data[3]^can->Data[4]^can->Data[5]^can->Data[6];
    Transmit(gDevType, gDevIdx, 0, can, 1);
    return 1;
}

int OdometerCmd(unsigned int odometerEnable,unsigned int odometerSetZero,unsigned int watchDog ,CAN_OBJ *can){
    memset(can, 0, sizeof(CAN_OBJ));    //全部置0
    can->ID = 0x18c4d6d0;
    can->SendType = 0 & 0xff;
    can->RemoteFlag = 0 & 0xff;
    can->ExternFlag = 1 & 0xff;
    can->DataLen = 8 & 0xff;
    can->Data[0] = odometerEnable & 0xff;
    can->Data[1] = odometerSetZero & 0xff;
    can->Data[6] = watchDog & 0xff;
    can->Data[7] = can->Data[0]^can->Data[1]^can->Data[2]^can->Data[3]^can->Data[4]^can->Data[5]^can->Data[6];
    Transmit(gDevType, gDevIdx, 0, can, 1);
    return 1;
}

int LightCmd(unsigned int headLight,unsigned int turnLightLeft,
        unsigned int turnLightRight,unsigned int watchDog ,CAN_OBJ *can){
    memset(can, 0, sizeof(CAN_OBJ));    //全部置0
    can->ID = 0x18c4d7d0;
    can->SendType = 0 & 0xff;
    can->RemoteFlag = 0 & 0xff;
    can->ExternFlag = 1 & 0xff;
    can->DataLen = 8 & 0xff;
    can->Data[0] = headLight & 0xff;
    can->Data[1] = turnLightLeft & 0xff;
    can->Data[2] = turnLightRight & 0xff;
    can->Data[6] = watchDog & 0xff;
    can->Data[7] = can->Data[0]^can->Data[1]^can->Data[2]^can->Data[3]^can->Data[4]^can->Data[5]^can->Data[6];
    Transmit(gDevType, gDevIdx, 0, can, 1);
    return 1;
}

int SpeakerCmd(unsigned int speaker,unsigned int watchDog ,CAN_OBJ *can){
    memset(can, 0, sizeof(CAN_OBJ));    //全部置0
    can->ID = 0x18c4d8d0;
    can->SendType = 0 & 0xff;
    can->RemoteFlag = 0 & 0xff;
    can->ExternFlag = 1 & 0xff;
    can->DataLen = 8 & 0xff;
    can->Data[0] = speaker & 0xff;
    can->Data[6] = watchDog & 0xff;
    can->Data[7] = can->Data[0]^can->Data[1]^can->Data[2]^can->Data[3]^can->Data[4]^can->Data[5]^can->Data[6];
    Transmit(gDevType, gDevIdx, 0, can, 1);
    return 1;
}

int init_can(){
    
    // ----- open device -------------------------------------------------

    if (!OpenDevice(gDevType, gDevIdx, 0)) {
        ROS_INFO("OpenDevice failed\n");
        return 0;
    }
    ROS_INFO("OpenDevice succeeded\n");    

    // ----- init & start -------------------------------------------------

    INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xffffffff;
    config.Filter = 0;
    config.Mode = 0;
    config.Timing0 = gBaud & 0xff;
    config.Timing1 = gBaud >> 8;

    int i;
    for (i = 0; i < MAX_CHANNELS; i++)
    {
        if ((gChMask & (1 << i)) == 0) continue;
        if (!InitCAN(gDevType, gDevIdx, i, &config))
        {
            printf("InitCAN(%d) failed\n", i);
            return 0;
        }
        printf("InitCAN(%d) succeeded\n", i);
        if (!StartCAN(gDevType, gDevIdx, i))
        {
            printf("StartCAN(%d) failed\n", i);
            return 0;
        }
        printf("StartCAN(%d) succeeded\n", i);
    }
    return 1;
}

// void canCmdCallback(const keyboard_control::move::ConstPtr &msg){
//     if (msg->driveSpeed > 0){
//         driveSpeed = msg->driveSpeed;
//         headLight = 1;
//     }
//     if (msg->driveSpeed == 0){
//         driveSpeed = 0;
//         braking = 0;
//         headLight = 0;
//     }
//     if (msg->driveSpeed < 0){
//         braking = 100 * msg->driveSpeed;
//         headLight = 0;
//     }

//     steering = msg->steering;
//     if (steering < 0){
//         turnLightLeft = 1;
//         turnLightRight = 0;
//     }
//     if (steering > 0){
//         turnLightRight = 1;
//         turnLightLeft = 0;
//     }
// }

// void canReceive(CAN_OBJ canRxDriveFeedback1, CAN_OBJ canRxDriveFeedback2, CAN_OBJ canRxWheelSpdFeedback1,
//                 CAN_OBJ canRxWheelSpdFeedback2, CAN_OBJ can_rx[10], float odm1, float odm2, float odm3,
//                 float speedFeedback, float wheelSpdFeedback_RL, float wheelSpdFeedback_RR){
void canReceive(){
    Receive(gDevType, gDevIdx, 0, &can_rx, 1, 10);
    ROS_INFO("---------------------------------------------------------");
    ROS_INFO("---BEOFRE:odm1:%.2f---odm2:%.2f---odm3:%.2f---", odm1, odm2, odm3);
    // for(int i=10; i>0; --i){
    //     if(can_rx[i].ID == 0x18C4D3EF){
    //         canRxDriveFeedback1 = canRxDriveFeedback2;
    //         canRxDriveFeedback2 = can_rx[i];
    //     }
    //     else if(can_rx[i].ID == 0x18C4D7EF){
    //         canRxWheelSpdFeedback1 = canRxWheelSpdFeedback2;
    //         canRxWheelSpdFeedback2 = can_rx[i];
    //     }
        if(can_rx.ID == 0x18C4D3EF){
            sec1 = sec2;
            sec2 = ros::Time::now().toSec();
            canRxDriveFeedback1 = canRxDriveFeedback2;
            canRxDriveFeedback2 = can_rx;
        }
        else if(can_rx.ID == 0x18C4D7EF){
            canRxWheelSpdFeedback1 = canRxWheelSpdFeedback2;
            canRxWheelSpdFeedback2 = can_rx;
        }
        speedFeedback = canRxDriveFeedback2.Data[1]*0.04;
        wheelSpdFeedback_RL = (canRxWheelSpdFeedback2.Data[1])*0.04-8;
        wheelSpdFeedback_RR = (canRxWheelSpdFeedback2.Data[3])*0.04-8;
        odm1 = odm1 + (sec2 - sec1)*speedFeedback;
        //odm1 = odm1 + (canRxDriveFeedback2.TimeStamp-canRxDriveFeedback1.TimeStamp)/1000000*speedFeedback;
        odm2 = odm2 + (canRxWheelSpdFeedback2.TimeStamp-canRxWheelSpdFeedback1.TimeStamp)/1000000*wheelSpdFeedback_RL;
        odm3 = odm3 + (canRxWheelSpdFeedback2.TimeStamp-canRxWheelSpdFeedback1.TimeStamp)/1000000*wheelSpdFeedback_RR;
        ROS_INFO("---sec1:%.6f,sec2:%.6f,sec2-sec1:%.6f---", sec1, sec2, sec2-sec1);
        ROS_INFO("---timeStamp2-timeStamp1:%d---", canRxDriveFeedback2.TimeStamp-canRxDriveFeedback1.TimeStamp);
        ROS_INFO("---speed:%.2f---wheelSpdRL:%.2f---wheelSpdRR:%.2f---", speedFeedback, wheelSpdFeedback_RL, wheelSpdFeedback_RR);
        //ROS_INFO("timeflag:%d", canRxDriveFeedback2.TimeFlag);
        ROS_INFO("---time:%d---", uint(canRxDriveFeedback2.TimeStamp));
        ROS_INFO("---odm1:%.2f---odm2:%.2f---odm3:%.2f---", odm1, odm2, odm3);
    //} 
}

void timer1Callback(const ros::TimerEvent){
    if (gear == 4) {
        canbus::RestartRobotMotion restartRobotMotionSrv;
        restartRobotMotionSrv.request.ToMove = true;
        restartRobotClientPtr->call(restartRobotMotionSrv);
        ROS_INFO("Feedback from robot: %d", restartRobotMotionSrv.response.ExecStatus);
    }
    gearEnable = 1;     //档位使能 0-disable 1-enable
    gear = 3;           //档位 02-后退 03-空档 04-前进
    steeringEnable = 1; //转向使能 0-disable 1-enable
    steering = 0;       //转角 0.043945度/bit offset-90
    driveEnable = 1;    //驱动使能 0-disable 1-enable
    driveSpeed = 0;     //目标车速 0.04m/s/bit   max10.24m/s
    brakingEnable = 1;  //制动使能 0-disable 1-enable
    braking = 0;       //制动踏板开度  0.390625%/bit   0-100
    parkingEnable = 1;  //驻车使能 0-disable 1-enable
    parking = 0;
}

void timer4CreepingCallback(const ros::TimerEvent){
    gearEnable = 1;     //档位使能 0-disable 1-enable
    gear = 3;           //档位 02-后退 03-空档 04-前进
    steeringEnable = 1; //转向使能 0-disable 1-enable
    steering = 0;       //转角 0.043945度/bit offset-90
    driveEnable = 1;    //驱动使能 0-disable 1-enable
    driveSpeed = 0;     //目标车速 0.04m/s/bit   max10.24m/s
    brakingEnable = 1;  //制动使能 0-disable 1-enable
    braking = 0;       //制动踏板开度  0.390625%/bit   0-100
    parkingEnable = 1;  //驻车使能 0-disable 1-enable
    parking = 0;

}

void timer2Callback(const ros::TimerEvent){
    moveforward();
}

void timerCallback1(const ros::TimerEvent){
    switch(cycle){
        case 0:
            gearEnable = 1;     //档位使能 0-disable 1-enable
            gear = 3;           //档位 02-后退 03-空档 04-前进
            steeringEnable = 1; //转向使能 0-disable 1-enable
            steering = 0;       //转角 0.043945度/bit offset-90
            driveEnable = 1;    //驱动使能 0-disable 1-enable
            driveSpeed = 0;     //目标车速 0.04m/s/bit   max10.24m/s
            brakingEnable = 1;  //制动使能 0-disable 1-enable
            braking = 0;       //制动踏板开度  0.390625%/bit   0-100
            parkingEnable = 1;  //驻车使能 0-disable 1-enable
            parking = 0;        //驻车请求 0-release 1-apply
            cycle++;
            break;
        case 1:
            gearEnable = 1;     //档位使能 0-disable 1-enable
            gear = 2;           //档位 02-后退 03-空档 04-前进
            steeringEnable = 1; //转向使能 0-disable 1-enable
            steering = 0;       //转角 0.043945度/bit offset-90
            driveEnable = 1;    //驱动使能 0-disable 1-enable
            driveSpeed = 0.2;   //目标车速 0.04m/s/bit   max10.24m/s
            brakingEnable = 1;  //制动使能 0-disable 1-enable
            braking = 0;        //制动踏板开度  0.390625%/bit   0-100
            parkingEnable = 1;  //驻车使能 0-disable 1-enable
            parking = 0;        //驻车请求 0-release 1-apply
            cycle++;
            break;
        case 2:
            gearEnable = 1;     //档位使能 0-disable 1-enable
            gear = 3;           //档位 02-后退 03-空档 04-前进
            steeringEnable = 1; //转向使能 0-disable 1-enable
            steering = 0;       //转角 0.043945度/bit offset-90
            driveEnable = 1;    //驱动使能 0-disable 1-enable
            driveSpeed = 0;     //目标车速 0.04m/s/bit   max10.24m/s
            brakingEnable = 1;  //制动使能 0-disable 1-enable
            braking = 0;       //制动踏板开度  0.390625%/bit   0-100
            parkingEnable = 1;  //驻车使能 0-disable 1-enable
            parking = 0;        //驻车请求 0-release 1-apply
            cycle++;
            break;
        case 3:
            gearEnable = 1;     //档位使能 0-disable 1-enable
            gear = 4;           //档位 02-后退 03-空档 04-前进
            steeringEnable = 1; //转向使能 0-disable 1-enable
            steering = 0;       //转角 0.043945度/bit offset-90
            driveEnable = 1;    //驱动使能 0-disable 1-enable
            driveSpeed = 0.24;     //目标车速 0.04m/s/bit   max10.24m/s
            brakingEnable = 1;  //制动使能 0-disable 1-enable
            braking = 0;       //制动踏板开度  0.390625%/bit   0-100
            parkingEnable = 1;  //驻车使能 0-disable 1-enable
            parking = 0;        //驻车请求 0-release 1-apply
            cycle = 0;
            break;
    }

}

bool MoveCarCb(canbus::MoveCarRequest &req, canbus::MoveCarResponse &res){
    if(req.ToMove){
        movebackward();
        res.ExecStatus = 1;
    }
    else
        res.ExecStatus = 0;
    return true;
}

bool CreepCb(canbus::AdjustCarPosRequest &req, canbus::AdjustCarPosResponse &res){
    res.ExecStatus = 1;
    if(req.MoveForward)
        creep(4);
    else
        creep(2);
    return true;
}


void moveforward(){
    gearEnable = 1;     //档位使能 0-disable 1-enable
    gear = 4;           //档位 02-后退 03-空档 04-前进
    steeringEnable = 1; //转向使能 0-disable 1-enable
    steering = 0;       //转角 0.043945度/bit offset-90
    driveEnable = 1;    //驱动使能 0-disable 1-enable
    driveSpeed = 0.2;     //目标车速 0.04m/s/bit   max10.24m/s
    brakingEnable = 1;  //制动使能 0-disable 1-enable
    braking = 0;       //制动踏板开度  0.390625%/bit   0-100
    parkingEnable = 1;  //驻车使能 0-disable 1-enable
    parking = 0;
    timer1.stop();
    timer1.setPeriod(ros::Duration(forwardtime));
    timer1.start();
}

void movebackward(){
    gearEnable = 1;     //档位使能 0-disable 1-enable
    gear = 2;           //档位 02-后退 03-空档 04-前进
    steeringEnable = 1; //转向使能 0-disable 1-enable
    steering = 0;       //转角 0.043945度/bit offset-90
    driveEnable = 1;    //驱动使能 0-disable 1-enable
    driveSpeed = 0.2;     //目标车速 0.04m/s/bit   max10.24m/s
    brakingEnable = 1;  //制动使能 0-disable 1-enable
    braking = 0;       //制动踏板开度  0.390625%/bit   0-100
    parkingEnable = 1;  //驻车使能 0-disable 1-enable
    parking = 0;
    timer1.stop();
    timer1.setPeriod(ros::Duration(backtime));
    timer1.start();
    timer2.stop();
    timer2.setPeriod(ros::Duration(backtime+10));
    timer2.start();
}

void creep(unsigned geararg){
    gearEnable = 1;     //档位使能 0-disable 1-enable
    gear = geararg;           //档位 02-后退 03-空档 04-前进
    steeringEnable = 1; //转向使能 0-disable 1-enable
    steering = 0;       //转角 0.043945度/bit offset-90
    driveEnable = 1;    //驱动使能 0-disable 1-enable
    driveSpeed = creepspeed;     //目标车速 0.04m/s/bit   max10.24m/s
    brakingEnable = 1;  //制动使能 0-disable 1-enable
    braking = 0;       //制动踏板开度  0.390625%/bit   0-100
    parkingEnable = 1;  //驻车使能 0-disable 1-enable
    parking = 0;
    timer4Creeping.stop();
    timer4Creeping.setPeriod(ros::Duration(creeptime));
    timer4Creeping.start();
}

int main(int argc,char *argv[]){
    ros::init(argc, argv, "can_cmd");
    ros::NodeHandle n;
    //ros::Subscriber sub = n.subscribe("move_cmd",1,canCmdCallback);
    ros::Rate loop_rate(100);   //单位Hz
    timer1 = n.createTimer(ros::Duration(99999), timer1Callback, true);
    timer2 = n.createTimer(ros::Duration(99999), timer2Callback, true);
    timer4Creeping = n.createTimer(ros::Duration(99999), timer4CreepingCallback, true);
    ros::ServiceServer MoveCar = n.advertiseService("move_car_service", MoveCarCb);
    ros::ServiceServer Creep = n.advertiseService("adjust_car_pos_service", CreepCb);
    ros::ServiceClient restartRobotClient = n.serviceClient<canbus::RestartRobotMotion>("restart_robot_motion_service");
    restartRobotClientPtr = &restartRobotClient;
    ROS_INFO("DevType=%d, DevIdx=%d, ChMask=0x%x, Baud=0x%04x, TxType=%d,\n",
        gDevType, gDevIdx, gChMask, gBaud, gTxType);

    if(1 != init_can()){
        ROS_INFO("Init USB-CAN failed");
        return 0;
    }
    CAN_OBJ can;
    while (ros::ok)
    {
        if (1 == GearCmd(gearEnable,gear,watchDog2,&can) &&
            1 == SteeringCmd(steeringEnable,steering,watchDog2,&can) &&
            1 == DriveCmd(driveEnable,driveSpeed,watchDog2,&can) &&
            1 == BrakingCmd(brakingEnable,braking,watchDog2,&can) &&
            1 == ParkingCmd(parkingEnable,parking,watchDog2,&can) &&
            1 == OdometerCmd(odometerEnable,odometerSetZero,watchDog2,&can) &&
            1 == LightCmd(headLight,turnLightLeft,turnLightRight,watchDog2,&can)  )
        ROS_INFO("canmsgs send success");
        ROS_INFO("\n"
            "gearEnable = %d \n"
            "gear = %d \n"
            "steeringEnable = %d \n"
            "steering = %.2f \n"
            "driveEnable = %d \n"
            "driveSpeed = %.2f \n"
            "brakingEnable %d \n"
            "braking = %.2f \n"
            "parkingEnable = %d \n"
            "parking = %d \n"
            "cycle = %d \n",gearEnable,gear,steeringEnable,steering,driveEnable,driveSpeed,
            brakingEnable,braking,parkingEnable,parking,cycle);
        //canReceive();

        if(watchDog == 15){ watchDog = -1;}
        watchDog++;
        watchDog2 = watchDog << 4;
        //ROS_INFO("0x%02x",watchDog2);
        ros::spinOnce();
        //ROS_INFO("speed:%f,braking:%f,steering:%f",driveSpeed,braking,steering);
        loop_rate.sleep();

    }
    CloseDevice(gDevType, gDevIdx);
    ROS_INFO("CloseDevice\n");
    return 1;
}
