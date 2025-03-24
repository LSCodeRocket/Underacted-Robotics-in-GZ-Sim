
#include <gz/sim/System.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/PID.hh>


#include "Gravity_Invert.hh"

#define STEP_TIME 0.001
#define TARGET_ANGLE 3.14159265 // meters

using namespace gz::sim;
using namespace std;

namespace simp_pend_ctrl {
    void Gravity_Invert::GetSDFData(const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm){
        this->jointName = "arm_joint";

        if (_sdf->HasElement("joint_name"))
        { this->jointName = _sdf->Get<std::string>("joint_name"); }

        this->jointEntity = _ecm.EntityByComponents(components::Name(this->jointName));

        if (this->jointEntity == kNullEntity)
        { gzerr << "Joint [" << this->jointName << "] not found!" << std::endl; return; }

        if (_sdf->HasElement("bob_mass"))
        { this->mass = _sdf->Get<double>("bob_mass"); }

        if (_sdf->HasElement("arm_length"))
        { this->length = _sdf->Get<double>("arm_length"); }

        if (_sdf->HasElement("theta_0"))
        { this->theta_0 = _sdf->Get<double>("theta_0"); }

        if (_sdf->HasElement("n"))
        { this->n = _sdf->Get<int>("n"); }

    }
    void Gravity_Invert::Configure(const Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        EntityComponentManager &_ecm,
                        EventManager &/*_eventMgr*/)
    {

        this->GetSDFData(_sdf, _ecm);

        _ecm.CreateComponent(this->jointEntity, components::JointPosition({0}));
        _ecm.CreateComponent(this->jointEntity, components::JointForceCmd({0}));

        auto posComp = _ecm.Component<components::JointPosition>(this->jointEntity);
        auto forceComp = _ecm.Component<components::JointForceCmd>(this->jointEntity);
        

        gzmsg << "Gravity Inverter initialized for joint: [" << this->jointName << "]" << std::endl;
        
    }

    
    void Gravity_Invert::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
    {
        if(_info.paused) {return;}
        if (this->jointEntity == kNullEntity) { return; }

        auto posComp = _ecm.Component<components::JointPosition>(this->jointEntity);
        auto forceComp = _ecm.Component<components::JointForceCmd>(this->jointEntity);

        double theta = posComp->Data().at(0) + this->theta_0;
        double force = 2 * this->mass * this->gravity * this->length * sin(theta);
        
        forceComp->Data()[0] = force;

        std::cout << "=============================================" << std::endl;
        std::cout << "N : " << n << std::endl;

        std::cout << "FORCE: " << -force << std::endl;
        std::cout << "THETA_0: " << this->theta_0 << std::endl;
        std::cout << "THETA: " << theta << std::endl;
        std::cout << "MASS: " << this->mass << std::endl;
        std::cout << "LENGTH: " << this->length << std::endl;
    }
}