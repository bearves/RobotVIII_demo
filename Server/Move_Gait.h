#ifndef MOVE_GAIT_H
#define MOVE_GAIT_H

#include <iostream>
#include <memory>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <map>
#include <string>
#include <stdlib.h>
#include <thread>

#include <Aris_Pipe.h>
#include <Aris_Core.h>
#include <Aris_Message.h>
#include <Aris_DynKer.h>
#include <Aris_Motion.h>
#include <Robot_Server.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>

using namespace Aris::Core;
using namespace std;
using namespace Aris::Control;


//static const double PI = 3.141592653589793;

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

//CWF
enum WALK_DIRECTION
{
	STOP,
	FORWARD,
	BACKWARD,
	RIGHTWARD,
	LEFTWARD,
	TURNLEFT,
	TURNRIGHT,
	FAST_TURNLEFT,
	FAST_TURNRIGHT
};


namespace ForceTask
{
	Aris::Core::MSG parseContinueMoveBegin(const std::string &cmd, const map<std::string, std::string> &params);
	Aris::Core::MSG parseContinueMoveJudge(const std::string &cmd, const map<std::string, std::string> &params);
	Aris::Core::MSG parseOpenDoorBegin(const std::string &cmd, const map<std::string, std::string> &params);
	Aris::Core::MSG parseOpenDoorJudge(const std::string &cmd, const map<std::string, std::string> &params);
	int continueMove(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
	int openDoor(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
	void StartRecordData();
	void inv3(double * matrix,double * invmatrix);
	void crossMultiply(double * vector_in1, double *vector_in2, double * vector_out);
	double dotMultiply(double *vector_in1, double *vector_in2);
	double norm(double * vector_in);

	struct CONTINUEMOVE_PARAM :public Robots::GAIT_PARAM_BASE
	{
		std::int32_t move_direction;
	};

	enum MoveState
	{
		None,
		PointLocate1,
		PointLocate2,
		LocateAjust,
		Forward,
		Backward,
		Rightward,
		Leftward,
		Follow,
		Downward,
		Upward,
		Pullhandle,
		Pushhandle,
		PrePush,
		Push,
	};

	enum PushState
	{
		now2Start,
		leftWalk,
		forwardWalk,
	};

	struct CM_RECORD_PARAM
	{
		double bodyPE_last[6];
		double bodyVel_last[6];

		double forceSum[6];
		double forceAvg[6]{0,0,0,0,0,0};
		double force[6];
	};

	struct OPENDOOR_PARAM :public CM_RECORD_PARAM
	{
		MoveState moveState;
		PushState pushState;
		int ret{0};
		std::int32_t count;
		std::int32_t countIter{0};
		Robots::WALK_PARAM walkParam;

		const double toolInR[3]{0,0.08,-0.385};
		double toolInG[3];

		//MoveState: PointLocation
		double pointLocation1[6];
		double pointLocation2[6];
		double pointLocation3[6];
		double location[3][3];

		//Door Location
		double planeYPR[3]{0,0,0};
		double handleLocation[3]{0,0,0};
		//startPE
		double beginPE[6];
		double vector0[3];
		double vector1[3];
		double vector2[3];

		//now2Start used twice
		double nowPE[6]; //used in Follow again
		double nowPee[18];
		double startPE[6];
		const int now2StartCount{2000};

		//MoveState: Follow
		double startPeeInB[18];
		double endPeeInB[18];
		const int followCount{2000};

		//MoveState: Downward
		bool downwardFlag;
		int downwardCount;

		//PushState
		double handlePE[6];
		double nowPm[4][4];
		double xNowInG[3];
		double yNowInG[3];
		double now2startDistance[3];
		double now2startDistanceModified[6]{0,0,0,0,0,0};
		double now2startDistanceReal[6]{0,0,0,0,0,0};
		double handle2startDistance[3];

		//pause
		MoveState moveState_last;
		int pauseCount{0};
		bool pauseFlag;
	};
};


extern PIPE<ForceTask::OPENDOOR_PARAM> openDoorPipe;
static std::thread openDoorThread;
extern PIPE<MOVES_PARAM> move2Pipe;
static std::thread move2Thread;

/*parse function*/
Aris::Core::MSG parseMove2(const std::string &cmd, const map<std::string, std::string> &params);
Aris::Core::MSG parseSwing(const std::string &cmd, const map<std::string, std::string> &params);
Aris::Core::MSG parseMoveWithRotate(const std::string &cmd, const map<std::string, std::string> &params);
Aris::Core::MSG parseCWF(const std::string &cmd, const std::map<std::string, std::string> &params);
Aris::Core::MSG parseCWFStop(const std::string &cmd, const std::map<std::string, std::string> &params);

/*operation function*/
int move2(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
int swing(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
int moveWithRotate(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
int continuousWalkWithForce(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
WALK_DIRECTION forceJudge(const double *force, const double *threshold);

#endif // MOVE_GAIT_H
