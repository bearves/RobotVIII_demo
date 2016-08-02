#include <rtdk.h>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <memory>

#include "NormalWalkWrapper.h"

using namespace NormalWalk;

double NormalWalkerWrapper::initialBodyPosition[6] = {0, 0, 0, 0, 0, 0};
double NormalWalkerWrapper::feetPosition[18] = 
    {0, 0, 0, 0, 0, 0, 
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0};

GAIT_CMD NormalWalkerWrapper::command = GAIT_CMD::NOCMD;
NormalWalker NormalWalkerWrapper::walker;

NormalWalkerWrapper::NormalWalkerWrapper()
{
}

NormalWalkerWrapper::~NormalWalkerWrapper()
{
}


void NormalWalkerWrapper::parseNormalWalking(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    NormalWalkerParam param;

    for(auto &i:params)
    {

        if(i.first=="i")
        {
            command = GAIT_CMD::INIT;
            msg.copyStruct(param);
            break;
        }
        else if(i.first=="b")
        {
            command = GAIT_CMD::START;
            break;
        }
        else if(i.first=="e")
        {
            command = GAIT_CMD::STOP;
            break;
        }
        else if(i.first=="u")
        {
            command = GAIT_CMD::SPEEDUP;
            break;
        }
        else if(i.first=="d")
        {
            command = GAIT_CMD::SPEEDDOWN;
            break;
        }
        else
        {
            std::cout<<"parse failed"<<std::endl;
        }
    }

    std::cout<<"finished parse"<<std::endl;
}

int NormalWalkerWrapper::normalWalking(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const NormalWalkerParam &>(param_in);
    double timeNow = param.count * 0.001;

    switch (command)
    {
        case GAIT_CMD::NOCMD:
            break;
        case GAIT_CMD::INIT:
            if (walker.GetState() == NormalWalker::GAIT_STATE::UNREADY || 
                walker.GetState() == NormalWalker::GAIT_STATE::STOPPED)
                walker.Initialize();
            rt_printf("GAIT CMD: INIT \n");
            break; 
        case GAIT_CMD::START:
            walker.Start(timeNow);
            rt_printf("GAIT CMD: START \n");
            break; 
        case GAIT_CMD::STOP:
            walker.Stop(timeNow);
            rt_printf("GAIT CMD: STOP \n");
            break; 
        case GAIT_CMD::SPEEDUP:
            {
                auto currentParam = walker.GetCurrentParameter();
                currentParam.velocity += 0.2;
                walker.AdjustGaitParam(timeNow, currentParam);
                rt_printf("GAIT CMD: SPEED UP \n");
            }
            break; 
        case GAIT_CMD::SPEEDDOWN:
            {
                auto currentParam = walker.GetCurrentParameter();
                currentParam.velocity -= 0.2;
                walker.AdjustGaitParam(timeNow, currentParam);
                rt_printf("GAIT CMD: SPEED DOWN\n");
            }
            break;
    }
    
    command = GAIT_CMD::NOCMD;

    //if (param.count % 5 == 0)
    //{
        //for(int i = 0; i < 6; i++)
        //{
            //diagnosticData.forceData[i] = rawForce[i];
        //}
        //for(int i = 0; i < 18; i++)
        //{
            //diagnosticData.svLeg[i] = feetPosition[i];
        //}
        //dataPipe.sendToNrt(diagnosticData);
    //}

    walker.GaitGenerator(timeNow, feetPosition);

    robot.SetPeb(initialBodyPosition);
    robot.SetPee(feetPosition, robot.ground());
    
    if (walker.GetState() == NormalWalker::GAIT_STATE::STOPPED)
        return 0;

    return 1;
}


