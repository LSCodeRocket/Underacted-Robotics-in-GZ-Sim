
#include <gz/sim/System.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/PID.hh>


#include "TEMPLATE.hh"

using namespace gz::sim;
using namespace std;

namespace NAMESPACE {
    void CLASS::Configure(const Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        EntityComponentManager &_ecm,
                        EventManager &/*_eventMgr*/) 
    {

    }

    
    void CLASS::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
    {
        if(_info.paused) {return;}
    }
}