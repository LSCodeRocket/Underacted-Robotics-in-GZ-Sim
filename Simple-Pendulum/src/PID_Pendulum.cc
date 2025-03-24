
#include <gz/sim/System.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/PID.hh>


#include "PID_Pendulum.hh"


#define STEP_TIME 0.001
#define TARGET_ANGLE 3.14159265 // meters

using namespace gz::sim;
using namespace std;

namespace simp_pend_ctrl {
    void PID_Pendulum::Configure(const Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        EntityComponentManager &_ecm,
                        EventManager &/*_eventMgr*/)
    {
        std::string jointName = "arm_joint";
        if (_sdf->HasElement("joint_name"))
        {
            jointName = _sdf->Get<std::string>("joint_name");
        }

        if (_sdf->HasElement("theta_0"))
        {
            this->theta_0 = _sdf->Get<double>("theta_0");
        }

        this->jointEntity = _ecm.EntityByComponents(components::Name(jointName));
        if (this->jointEntity == kNullEntity)
        {
            gzerr << "Joint [" << jointName << "] not found!" << std::endl;
            return;
        }

        double p = 10.0;
        double i = 0.0;
        double d = 1.0;

        if (_sdf->HasElement("p_gain"))
        {
            p = _sdf->Get<double>("p_gain");
        }
        if (_sdf->HasElement("i_gain"))
        {
            i = _sdf->Get<double>("i_gain") ;
        }
        if (_sdf->HasElement("d_gain"))
        {
            d = _sdf->Get<double>("d_gain");
        }

        this->pid.Init(p, i, d, 20.0, -20.0, 1000.0, -1000.0); // PID parameters

        if (_sdf->HasElement("target_position"))
        {
            this->targetPosition = _sdf->Get<double>("target_position");
        }
        else {
            this->targetPosition = 0;
        }

        gzmsg << "Pendulum controller initialized for joint: [" << jointName << "]" << std::endl;
    }

    
    void PID_Pendulum::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
    {
        if(_info.paused) {return;}
        if (this->jointEntity == kNullEntity)
        {
            return;
        }

        auto posComp = _ecm.Component<components::JointPosition>(this->jointEntity);
        auto forceComp = _ecm.Component<components::JointForceCmd>(this->jointEntity);

        if (!posComp)
        {
            _ecm.CreateComponent(this->jointEntity, components::JointPosition({0}));
            gzerr << "Joint position component not found." << std::endl;
            return;
        }

        double currentPosition = posComp->Data().at(0) + this->theta_0;
        double error = (this->targetPosition - currentPosition);
        
        double force = this->pid.Update(error, _info.dt);

        if (!forceComp)
        {
            _ecm.CreateComponent(this->jointEntity, components::JointForceCmd({force}));
        }
        else
        {
            forceComp->Data()[0] = -force;
            std::cout<< error << std::endl;
        }
    }
}