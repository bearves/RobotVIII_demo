#ifndef MOVE_GAIT_H
#define MOVE_GAIT_H

#endif // MOVE_GAIT_H

#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <map>
#include <string>
#include <stdlib.h>

#include <Aris_Core.h>
#include <Aris_Message.h>
#include <Aris_DynKer.h>
#include <Aris_Motion.h>
#include <Robot_Server.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>

using namespace Aris::Core;
using namespace std;



#ifndef PI
#define PI 3.141592653589793
#endif

struct MOVES_PARAM :public Robots::GAIT_PARAM_BASE
{
    double targetPee[18]{0};
    double targetBodyPE[6]{0};
    std::int32_t periodCount;
    int comID; //移动的部件（component）序号
    bool isAbsolute{false}; //用于判断移动命令是绝对坐标还是相对坐标
};

struct SWING_PARAM :public Robots::GAIT_PARAM_BASE
{
    double centreP[3]{0};
    double swingRad;
    std::int32_t periodCount;
};

struct MOVEWITHROTATE_PARAM :public Robots::GAIT_PARAM_BASE
{
	double targetBodyPE213[6]{0};
	std::int32_t totalCount;
};

struct CONTINUEMOVE_PARAM :public Robots::GAIT_PARAM_BASE
{
    std::int8_t move_direction[6];
	//bool isStart;
};

struct CM_LAST_PARAM
{
	double bodyPE_last[6]{0};
	double bodyVel_last[6]{0};
};

/*parse function*/
Aris::Core::MSG parseMove2(const std::string &cmd, const map<std::string, std::string> &params);
Aris::Core::MSG parseSwing(const std::string &cmd, const map<std::string, std::string> &params);
Aris::Core::MSG parseMoveWithRotate(const std::string &cmd, const map<std::string, std::string> &params);
//Aris::Core::MSG parseContinueMoveBegin(const std::string &cmd, const map<std::string, std::string> &params);
Aris::Core::MSG parseContinueMoveJudge(const std::string &cmd, const map<std::string, std::string> &params);

/*operation function*/
int move2(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
int swing(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
int moveWithRotate(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
//int continueMoveBegin(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
int continueMoveJudge(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
