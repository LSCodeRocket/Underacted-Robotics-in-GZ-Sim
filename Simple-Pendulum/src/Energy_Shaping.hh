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


namespace simp_pend_ctrl
{
    class Energy_Shaping :
        public gz::sim::System, 
        public gz::sim::ISystemConfigure, 
        public gz::sim::ISystemPreUpdate,
        public gz::sim::ISystemPostUpdate
    {
    public:
        Energy_Shaping() {};
        ~Energy_Shaping() {};

        void Configure(const Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            EntityComponentManager &_ecm,
                            EventManager &/*_eventMgr*/) override; 

        void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;
        void PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm) override;

    private:

        Entity jointEntity = kNullEntity;

        double mass = 10.0;
        double length = 10.0;

        double theta_0 = 0.0;

        int n = 0;

        const double gravity = 9.81;

        std::string jointName;
        std::string logFileName;

        std::ofstream logFile;


        void GetSDFData(const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm);

    };

}

GZ_ADD_PLUGIN(
    simp_pend_ctrl::Energy_Shaping,
    gz::sim::System, 
    simp_pend_ctrl::Energy_Shaping::ISystemConfigure,
    simp_pend_ctrl::Energy_Shaping::ISystemPreUpdate,
    simp_pend_ctrl::Energy_Shaping::ISystemPostUpdate)

#endif