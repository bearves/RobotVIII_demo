#include "Move_Gait.h"
#include "rtdk.h"

#ifdef UNIX
#include "rtdk.h"
#endif
#ifdef WIN32
#define rt_printf printf
#endif

#include <cstring>
#include <cmath>
#include <algorithm>
#include <memory>

void parseMoveWithRotate(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    MoveRotateParam param;

    for(auto &i:params)
    {
        if(i.first=="u")
        {
            param.targetBodyPE213[0]=stod(i.second);
        }
        else if(i.first=="v")
        {
            param.targetBodyPE213[1]=stod(i.second);
        }
        else if(i.first=="w")
        {
            param.targetBodyPE213[2]=stod(i.second);
        }
        else if(i.first=="yaw")
        {
            param.targetBodyPE213[3]=stod(i.second)*PI/180;
        }
        else if(i.first=="pitch")
        {
            param.targetBodyPE213[4]=stod(i.second)*PI/180;
        }
        else if(i.first=="roll")
        {
            param.targetBodyPE213[5]=stod(i.second)*PI/180;
        }
        else if(i.first=="totalCount")
        {
            param.totalCount=std::stoi(i.second);
        }
        else
        {
            std::cout<<"parse failed"<<std::endl;
        }
    }

    msg.copyStruct(param);

    std::cout<<"finished parse"<<std::endl;
}

int moveWithRotate(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const MoveRotateParam &>(param_in);

    static double beginBodyPE213[6];
    static double pEE[18];
    if(param.count==0)
    {
        robot.GetPeb(beginBodyPE213,"213");
        robot.GetPee(pEE);
    }

    double realBodyPE213[6];
    for(int i=0;i<6;i++)
    {
        double s = -(param.targetBodyPE213[i] / 2)*cos(PI * (param.count + 1) / param.totalCount ) + param.targetBodyPE213[i] / 2;
        realBodyPE213[i]=beginBodyPE213[i]+s; //target of current ms
    }

    double pBody[6];

    robot.SetPeb(realBodyPE213,"213");
    robot.SetPee(pEE);

    return param.totalCount - param.count - 1;
}

