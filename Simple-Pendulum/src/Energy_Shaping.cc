
#include <gz/sim/System.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointForce.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/PID.hh>
#include <fstream>

#define STEP_SIZE 0.001
#define LOG_TIME 8

#include "Energy_Shaping.hh"

using namespace gz::sim;
using namespace std;

namespace simp_pend_ctrl {
    void Energy_Shaping::GetSDFData(const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm){
        this->jointName = "arm_joint";

        if (_sdf->HasElement("joint_name"))
        { this->jointName = _sdf->Get<std::string>("joint_name"); }

        if (_sdf->HasElement("log_file_name"))
        { this->logFileName = _sdf->Get<std::string>("log_file_name"); }


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
    void Energy_Shaping::Configure(const Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        EntityComponentManager &_ecm,
                        EventManager &/*_eventMgr*/)
    {

        this->GetSDFData(_sdf, _ecm);

        _ecm.CreateComponent(this->jointEntity, components::JointPosition({0}));
        _ecm.CreateComponent(this->jointEntity, components::JointVelocity({0}));
        _ecm.CreateComponent(this->jointEntity, components::JointForceCmd({0}));

        this->logFile.open(this->logFileName);

        this->logFile << "TIME,POSITION,VELOCITY\n";

        gzmsg << "Energy_Shaper initialized for joint: [" << this->jointName << "]" << std::endl;
        
    }

    void Energy_Shaping::PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm)
    {
        if(_info.paused) {return;}
        if (this->jointEntity == kNullEntity) { return; }
        if(_info.simTime.count()*1e-9 >= LOG_TIME) { return; }
        
        auto posComp = _ecm.Component<components::JointPosition>(this->jointEntity);
        auto velComp = _ecm.Component<components::JointVelocity>(this->jointEntity);

        this->logFile << _info.simTime.count()*1e-9 << "," << posComp->Data()[0] + this->theta_0 << "," << velComp->Data()[0] 
            << "," << this->mass << "," << this->length << "\n";

        if(_info.simTime.count()*1e-9 != LOG_TIME) { return; }

        this->logFile.close();

        

    }
    
    void Energy_Shaping::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
    {
        if(_info.paused) {return;}
        if (this->jointEntity == kNullEntity) { return; }

        auto posComp = _ecm.Component<components::JointPosition>(this->jointEntity);
        auto velComp = _ecm.Component<components::JointVelocity>(this->jointEntity);
        auto forceComp = _ecm.Component<components::JointForceCmd>(this->jointEntity);

        double theta = posComp->Data().at(0) + this->theta_0;
        double theta_dot = velComp->Data().at(0);
        double curr_energy = this->mass * pow(this->length * theta_dot, 2)/2 - this->mass * this->gravity * this->length * cos(theta);
        
        double k = 50;

        double force = -k*theta_dot * (curr_energy - this->mass * this->gravity * this->length);
        
        forceComp->Data()[0] = force;

        std::cout << "=============================================" << std::endl;
        std::cout << "N : " << n << std::endl;

        std::cout << "TIME: " << _info.simTime.count()*1e-9 << std::endl;
        std::cout << "FORCE: " << -force << std::endl;
        std::cout << "THETA_0: " << this->theta_0 << std::endl;
        std::cout << "THETA: " << theta << std::endl;
        std::cout << "MASS: " << this->mass << std::endl;
        std::cout << "LENGTH: " << this->length << std::endl;
    }
}