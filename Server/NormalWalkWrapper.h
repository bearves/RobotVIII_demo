#ifndef NORMAL_WALK_WRAPPER_H
#define NORMAL_WALK_WRAPPER_H

#include <thread>
#include <functional>
#include <cstdint>
#include <map>

#include <aris.h>
#include <aris_control_pipe.h>
#include <Robot_Type_I.h>
#include <Robot_Gait.h>

#include "NormalWalk.h"

using namespace aris::control;

namespace NormalWalk
{
    enum class GAIT_CMD
    {
        NOCMD     = 0,
        INIT      = 1,
        START     = 2,
        STOP      = 3,
        SPEEDUP   = 4,
        SPEEDDOWN = 5
    };

    struct NormalWalkerParam final :public aris::server::GaitParamBase
    {
    };

    // Wrap the walker to adapt to the interface of ARIS
    class NormalWalkerWrapper
    {
        public:
            NormalWalkerWrapper();
            ~NormalWalkerWrapper();
            static void parseNormalWalking(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
            static int normalWalking(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

        private:
            static double initialBodyPosition[6];
            static double feetPosition[18];
            static GAIT_CMD command;
            static NormalWalker walker;
    };

}


#endif 
