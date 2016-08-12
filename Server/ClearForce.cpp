#include "ClearForce.h"

namespace ClearForce
{

void parseClearForce(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    ClearForceParam param;
    msg.copyStruct(param);
    std::cout<<"finished parse"<<std::endl;
}

int clearForce(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const ClearForceParam&>(param_in);
    auto * cpParam = &param;
    ClearForceParam* pParam = const_cast<ClearForceParam *>(cpParam);

    for(int i = 0; i < 6; i++)
    {
        auto* pVecForceData = const_cast<vector<aris::control::EthercatForceSensor::Data> *>(pParam->force_data);
        pVecForceData->at(i).isZeroingRequested = true;
    }

    rt_printf("Clear force invoked\n");
    return 0;
}
}
