﻿#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <aris.h>
#include <Robot_Type_I.h>

#ifdef WIN32
#define rt_printf printf
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

#include "NormalWalkWrapper.h"


int main(int argc, char *argv[])
{
    NormalWalk::NormalWalkerWrapper wrapper;
    std::string xml_address;

    if (argc <= 1)
    {
        std::cout << "you did not type in robot name, in this case ROBOT-VIII will start" << std::endl;
        xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_VIII/Robot_VIII.xml";
    }
    else if (std::string(argv[1]) == "III")
    {
        xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "VIII")
    {
        xml_address = "../../resource/Robot_VIII.xml";
    }
    else
    {
        throw std::runtime_error("invalid robot name, please type in III or VIII");
    }

    auto &rs = aris::server::ControlServer::instance();

    rs.createModel<Robots::RobotTypeI>();
    rs.loadXml(xml_address.c_str());
    rs.addCmd("en", Robots::basicParse, nullptr);
    rs.addCmd("ds", Robots::basicParse, nullptr);
    rs.addCmd("hm", Robots::basicParse, nullptr);
    rs.addCmd("rc", Robots::recoverParse, Robots::recoverGait);
    rs.addCmd("wk", Robots::walkParse, Robots::walkGait);
    rs.addCmd("ro", Robots::resetOriginParse, Robots::resetOriginGait);

    rs.addCmd("nw",wrapper.parseNormalWalking, wrapper.normalWalking);
    rs.open();

    rs.setOnExit([&]()
            {
            aris::core::XmlDocument xml_doc;
            xml_doc.LoadFile(xml_address.c_str());
            auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
            if (!model_xml_ele)throw std::runtime_error("can't find Model element in xml file");
            rs.model().saveXml(*model_xml_ele);

            aris::core::stopMsgLoop();
            });
    aris::core::runMsgLoop();
    return 0;
}
