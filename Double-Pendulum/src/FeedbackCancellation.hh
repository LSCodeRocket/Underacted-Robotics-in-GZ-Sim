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

namespace dbl_pend_ctrl
{
    class FeedbackCancellation :
        public gz::sim::System, 
        public gz::sim::ISystemConfigure, 
        public gz::sim::ISystemPreUpdate
    {
    public:
        FeedbackCancellation() {};
        ~FeedbackCancellation() {};

        void Configure(const Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            EntityComponentManager &_ecm,
                            EventManager &/*_eventMgr*/) override; 

        void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

    private:

            Entity joint_1;
            Entity joint_2;

            double m_1;
            double m_2;

            double l_1;
            double l_2;

            double theta_1_0;
            double theta_2_0;

            double g = 9.81;


    };

}

GZ_ADD_PLUGIN(
    dbl_pend_ctrl::FeedbackCancellation,
    gz::sim::System, 
    dbl_pend_ctrl::FeedbackCancellation::ISystemConfigure,
    dbl_pend_ctrl::FeedbackCancellation::ISystemPreUpdate)

#endif