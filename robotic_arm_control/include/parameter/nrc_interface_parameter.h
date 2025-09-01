#ifndef INCLUDE_API_NRC_INTERFACE_PARAMETER_H_
#define INCLUDE_API_NRC_INTERFACE_PARAMETER_H_

#include <string>

struct ToolParam {
    double X;                       //X轴偏移方向
    double Y;                       //Y轴偏移方向
    double Z;                       //Z轴偏移方向
    double A;                       //绕A轴旋转
    double B;                       //绕B轴旋转
    double C;                       //绕C轴旋转
    double payloadMass;             //负载质量
    double payloadInertia;          //负载惯性
    double payloadMassCenter_X;     //负载质心X
    double payloadMassCenter_Y;     //负载质心Y
    double payloadMassCenter_Z;     //负载质心Z
};

struct RobotDHParam
{
    double L1{0};
    double L2{0};
    double L3{0};
    double L4{0};
    double L5{0};
    double L6{0};
    double L7{0};
    double L8{0};
    double L9{0};
    double L10{0};
    double L11{0};
    double L12{0};
    double L13{0};
    double L14{0};
    double L15{0};
    double L16{0};
    double L17{0};
    double L18{0};
    double L19{0};
    double L20{0};

    std::string Couple_Coe_1_2;
    std::string Couple_Coe_2_3;
    std::string Couple_Coe_3_2;
    std::string Couple_Coe_3_4;
    std::string Couple_Coe_4_5;
    std::string Couple_Coe_4_6;
    std::string Couple_Coe_5_6;

    double dynamicLimit_max{0};
    double dynamicLimit_min{0};


    double pitch{0};//螺距
    double sliding_lead_value{0};//滑动电动缸导程,酒槽机型用
    double uplift_lead_value{0};//顶升电动缸导程,酒槽机型用
    double spray_distance{0};//喷料距离,酒槽机型用

    double threeAxisDirection{0};//3轴方向
    double fiveAxisDirection{0};//五轴方向

    double twoAxisConversionRatio{0};
    double threeAxisConversionRatio{0};
    double amplificationRatio;

    double conversionratio_x{0};
    double conversionratio_y{0};
    double conversionratio_z{0};

    double conversionratio_J1{0};  //1轴转换比 五轴混动
    double conversionratio_J2{0};
    double conversionratio_J3{0};

    int upsideDown{0};
};

struct ServoMovePara {
	bool clearBuffer;                       ///< 是否清除之前发送的，未开始插补计算的点位
	int targetMode;                         ///< 0-独立点 1-连续轨迹
	int sendMode;                           ///< 0-一次传输完全部轨迹 1-一次传输部分点位
	int runMode;                            ///< 0-接收完再运动 1-边接受边运动
	int sum;                                ///< 总传输次数
	int count;                              ///< 当前是第几次
	int coord;                              ///< 0-关节 1-直角
	int extMove;                            ///< 0- 1-
	int size;                               ///< 本次传输的点位数
	std::vector<std::vector<double>> pos;   ///< 二维数组，一维表示本次传输的点位数，二维长度为7，各个关节角度或笛卡尔坐标
        std::vector<std::vector<double>> axisvel; ///< 二维数组，一维表示本次传输的点位数，二维长度为7，各个轴的速度
        std::vector<std::vector<double>> axisacc; ///< 二维数组，一维表示本次传输的点位数，二维长度为7，各个轴的加速度
	std::vector<double> timeStamp;          ///< 长度为本次传输的点位数，表示到达该点位的时间，单位ms
};

struct RobotJointParam {
    double reducRatio;          //减速比
    int encoderResolution;      //编码器位数
    double posSWLimit;          //轴正限位
    double negSWLimit;          //轴反限位
    double ratedRotSpeed;       //电机额定正转速
    double ratedDeRotSpeed;     //电机额定反转速
    double maxRotSpeed;         //电机最大正转速
    double maxDeRotSpeed;       //电机最大反转速
    double ratedVel;            //额定正速度
    double deRatedVel;          //额定反速度
    double maxAcc;              //最大加速度
    double maxDecel;            //最大减速度
    int direction;              //模型方向，1：正向，-1：反向
};

struct CollisionPara {
	std::vector<double> collisionDetection_run;    ///<数组，碰撞检测阈值（指令），第几位为第几轴的碰撞检测阈值，参数范围：1≤vector_collisionDetection_run≤10000
	std::vector<double> collisionDetection_teach;  ///<数组，碰撞检测阈值（点动），第几位为第几轴的碰撞检测阈值，参数范围：1≤vector_collisionDetection_teach≤10000
	double position_delay_time_ms_value{0.0};      ///<//指令位置响应时间，参数范围：0<position_delay_time_ms_value≤99
	double error_enable_time_ms_value{0.0};	       ///<误差允许时间，参数范围：0≤error_enable_time_ms_value≤99
	unsigned int axisum{6};                        ///<机器人轴数，默认为六轴机器人
};

#endif /* INCLUDE_API_NRC_INTERFACE_PARAMETER_H_ */
