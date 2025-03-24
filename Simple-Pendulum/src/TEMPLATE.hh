#ifndef SYSTEM_PLUGIN_GRAVITY_INVERT_HH_
#define SYSTEM_PLUGIN_GRAVITY_INVERT_HH_

#include <gz/sim/System.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/PID.hh>


using namespace gz::sim;

namespace NAMESPACE 
{
    class CLASS :
        public gz::sim::System, 
        public gz::sim::ISystemConfigure, 
        public gz::sim::ISystemPreUpdate
    {
    public:
        CLASS() {};
        ~CLASS() {};

        void Configure(const Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            EntityComponentManager &_ecm,
                            EventManager &/*_eventMgr*/) override; 

        void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

    private:

    };

}

GZ_ADD_PLUGIN(
    NAMESPACE::CLASS,
    gz::sim::System, 
    NAMESPACE::CLASS::ISystemConfigure,
    NAMESPACE::CLASS::ISystemPreUpdate)

#endif