#ifndef SYSTEM_PLUGIN_PID_PENDULUM_HH_
#define SYSTEM_PLUGIN_PID_PENDULUM_HH_

#include <gz/sim/System.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/PID.hh>

using namespace gz::sim;

namespace simp_pend_ctrl
{
    class PID_Pendulum :
        public gz::sim::System, 
        public gz::sim::ISystemConfigure, 
        public gz::sim::ISystemPreUpdate
    {
    public:
        PID_Pendulum() {};
        ~PID_Pendulum() {};

        void Configure(const Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            EntityComponentManager &_ecm,
                            EventManager &/*_eventMgr*/) override; 

        void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

    private:
        Entity jointEntity = kNullEntity;
        gz::math::PID pid;
        double targetPosition = 0;
        double theta_0 = 0;
    };

}


GZ_ADD_PLUGIN(
    simp_pend_ctrl::PID_Pendulum,
    gz::sim::System, 
    simp_pend_ctrl::PID_Pendulum::ISystemConfigure,
    simp_pend_ctrl::PID_Pendulum::ISystemPreUpdate)

#endif