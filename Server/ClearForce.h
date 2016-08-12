#ifndef CLEAR_FORCE_H
#define CLEAR_FORCE_H

#include "aris.h"
#include "rtdk.h"
#include "Robot_Type_I.h"

namespace ClearForce
{
    struct ClearForceParam final :public aris::server::GaitParamBase
    {
    };

    static void parseClearForce(
            const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);

    // Only to demostrate how to clear force,
    // you should add this snippets to your own gait planner function according to your situation
    static int clearForce(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);
}

#endif
