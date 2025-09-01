#ifndef INCLUDE_API_NRC_INTERFACE_DEFINE_H_
#define INCLUDE_API_NRC_INTERFACE_DEFINE_H_

#include <string>
#include <vector>

#if	defined(_WIN32) || defined(WIN32)   //windows
#define EXPORT_API __declspec(dllexport)
#else
#define EXPORT_API
#endif

const int const_robotNum = 1;
using SOCKETFD = int;

enum Result {
    EXCEPTION = -5,
    OPERATION_NOT_ALLOWED = -4,
    PARAM_ERR = -3,
    DISCONNECT = -2,
    RECEIVE_FAILED = -1,
    SUCCESS = 0
};

enum class PosType {
	data = 0,    //自定义数组
    PType = 1,
    E_TYPE = 2,
    RP_TYPE = 3,
    AP_TYPE = 4,
	GPType = 5,  //系统内置的全局点位 GP001 只含机器人本体的点位
	GEType = 6   //系统内置的全局点位 GE001 含外部轴的机器人点位
};

//移动指令参数  单独的Movj只能使用数值   作业文件运行模式只支持使用变量   
struct MoveCmd {
	PosType targetPosType{PosType::data};
	std::vector<double> targetPosValue; //如果posType=PosType::data 为自定义数组,需要设置该向量值,前7位为本体值，后7位为外部轴
	std::string targetPosName{"GP0001"}; //如果posType为内置点位,需要设置该值,如 posType=PosType::GPType;posName=“GP0001”;
    int coord{0};
	double velocity{50};
	double acc{50};
	double dec{50};
	int pl{0};
	int time{0}; //提前执行时间
	int toolNum{0};
	int userNum{0};
    int posidtype{0}; //0:P GP 一级变量类型; 1:P[I001] GP[I001] 二级变量类型  需要使用二级类型时,实例如: targetPosType = PosType::GPType; posName=“GP￥I001”; posidtype=1; 插入到作业文件中的变量即为GP[I001]
    int configuration{0}; //形态
    MoveCmd() : targetPosValue(14) {};
};

enum LogicType {
	EQUAL_TO = 1,   //  ==
	LESS,           //  <
	GREATER,        //  >
	LESS_EQUAL,     //  <=
	GREATER_EQUAL,  //  >=
	NOT_EQUAL_TO  
};

struct ParaGroup {
	double data{0.0}; //数值型给这个赋值
	int secondvalue{0};  //变量型是否存在二级变量 1存在 0不存在 例如 "DOUT[I001]" 这种填1
	int value{0}; //0 数值型  1变量型
	std::string varname{""}; //变量型给这个写变量名 例如 "I001" , "DOUT[I001]"
};

struct Condition {
    std::string desValue{"0"};
    std::string key{"0"};
	LogicType logicType{LogicType::EQUAL_TO};
	ParaGroup paraGroupOne;
	ParaGroup paraGroupTwo;
};

struct PositionData
{
   std::string key{""};//变量名  AP0001
   std::vector<double> posData;  //坐标数据 长度7
   int coord{0};  // 0关节  1直角  2工具  3用户
   int toolNum{0};
   int userNum{0};
   PosType type{PosType::data}; // 参考PosType根据变量名选择 AP是4

    static PosType determineType(const std::string& key)
    {
        if (key.substr(0, 1) == "P") {
            return PosType::PType;
        }
        return PosType::data; 
    }
};

struct IOCommandParams {
    std::string groupType;    // 设置输出路数 可选路数 OT#:1路输出 OGH#:4路输出 OG#：8路输出
    int errorHanding;         // 0:输出值保存 1:计时结束停止
    ParaGroup paraGroupNum;   // 设置输出IO版和输出组号 选择OT# 范围[1,16] 选择OGH# 范围[1,16] 选择OG# 范围[1,8]
    ParaGroup paraGroupTime;  // 时间 [0,9999]S
    ParaGroup paraGroupValue; // 输出值 选择OT# 端口范围1 选择OGH# 端口范围[1,4] 选择OG# 范围[1,8]
                              // 勾选端口1:1 端口2:2 端口3:4 端口4:8 端口5:16 端口6:32 端口7:64 端口8:128
                              // 例如勾选端口1和端将口2和端口3 输入 :7
};

struct OffsetCommandParam
{
	std::vector<double> calData;//轴参数 手填值范围[-50000，50000],
    std::string coord {"BF"}; //位置变量坐标系 RF-关节坐标 BF-直角坐标 UF用户坐标
    std::vector<double> datatype;  //变量类型 0-手填值 1 变量类型  ：例如 caldata是["1","2","3","I001","D001","GI001","GD001"]  这个datatype就是 [0,0,0,1,1,1,1]
    int tool {-1};
    int user {-1};
};

enum class RunType {
	PP = 0,
	CSP,
	PV,
	CSV,
	PT,
};

struct AXISPostion {
	int axis_set_num;    ///< 轴号
	int axis_set_type;   ///< 0-绝对位置 1-相对位置
	RunType control_axis_run_type = RunType::PP;
	double axis_target_pos;
	double axis_set_speed;
	double axis_set_acc;
	double axis_set_dec;
};

struct AxisVelocity {
	int axis_set_num;  ///< 轴号
	RunType control_axis_run_type = RunType::PV;  ///< PV or CSV
	double axis_set_velocity;
	double axis_set_acc;
	double axis_set_dec;
};

struct AxisTorque {
	int axis_set_num;  ///< 轴号
	double axis_set_torque;
	double axis_torque_slope;
};

#endif /* INCLUDE_API_NRC_INTERFACE_DEFINE_H_ */



