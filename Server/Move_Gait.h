#ifndef MOVE_GAIT_H
#define MOVE_GAIT_H

#include <thread>
#include <functional>
#include <cstdint>
#include <map>

#include <aris.h>
#include <aris_control_pipe.h>
#include <Robot_Type_I.h>
#include <Robot_Gait.h>

using namespace aris::control;

namespace NormalWalker
{

struct MoveRotateParam final :public aris::server::GaitParamBase
{
    double targetBodyPE213[6]{0};
    std::int32_t totalCount;
};

class NormalWalkerInterface
{

    void parseNormalWalking(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
    int normalWalking(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);
};

}


#endif // MOVE_GAIT_H
