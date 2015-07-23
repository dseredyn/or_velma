////
// CollisionCheckerPlugin.cpp
//
//  Created on: Jul 23, 2015
//      Author: dseredyn
////
#include "VelmaQ5Q6CollisionChecker.h"
#include <openrave/plugin.h>

using namespace OpenRAVE;
using namespace or_velma;

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    if (type == OpenRAVE::PT_CollisionChecker && interfacename == "or_velma_q5q6_checker")
    {
        VelmaQ5Q6CollisionChecker *checker = new VelmaQ5Q6CollisionChecker(penv, penv->GetCollisionChecker());
        return OpenRAVE::InterfaceBasePtr(checker);
    }

    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{
    info.interfacenames[OpenRAVE::PT_CollisionChecker].push_back("or_velma_q5q6_checker");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    return;
}
