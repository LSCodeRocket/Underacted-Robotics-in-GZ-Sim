
#include <gz/sim/System.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/PID.hh>


#include "PIDController.hh"

using namespace gz::sim;
using namespace std;

namespace dbl_pend_ctrl {
    void PIDController::Configure(const Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        EntityComponentManager &_ecm,
                        EventManager &/*_eventMgr*/) 
    {
        std::string joint_1_name;
        std::string joint_2_name;

        if(_sdf->HasElement("joint_1_name")){ joint_1_name = _sdf->Get<std::string>("joint_1_name"); }
        if(_sdf->HasElement("joint_2_name")){ joint_2_name = _sdf->Get<std::string>("joint_2_name"); }

        if(_sdf->HasElement("P_gain_1")){ P_1 = _sdf->Get<double>("P_gain_1"); }
        if(_sdf->HasElement("P_gain_2")){ P_2 = _sdf->Get<double>("P_gain_2"); }

        if(_sdf->HasElement("I_gain_1")){ I_1 = _sdf->Get<double>("I_gain_1"); }
        if(_sdf->HasElement("I_gain_2")){ I_2 = _sdf->Get<double>("I_gain_2"); }

        if(_sdf->HasElement("D_gain_1")){ D_1 = _sdf->Get<double>("D_gain_1"); }
        if(_sdf->HasElement("D_gain_2")){ D_2 = _sdf->Get<double>("D_gain_2"); }

        if(_sdf->HasElement("theta_1_0")){ theta_1_0 = _sdf->Get<double>("theta_1_0"); }
        if(_sdf->HasElement("theta_2_0")){ theta_2_0 = _sdf->Get<double>("theta_2_0"); }


        pid_1.Init(P_1, I_1, D_1, 50, -50, 1000, -1000);
        pid_2.Init(P_2, I_2, D_2, 20, -20, 100, -100);


        joint_1 = _ecm.EntityByComponents(components::Name(joint_1_name));
        joint_2 = _ecm.EntityByComponents(components::Name(joint_2_name));

        _ecm.CreateComponent(joint_1, components::JointPosition({0}));
        _ecm.CreateComponent(joint_1, components::JointForceCmd({0}));
        _ecm.CreateComponent(joint_1, components::JointPositionReset({theta_1_0}));

        _ecm.CreateComponent(joint_2, components::JointPosition({0}));
        _ecm.CreateComponent(joint_2, components::JointForceCmd({0}));
        _ecm.CreateComponent(joint_2, components::JointPositionReset({theta_2_0}));

    }

    
    void PIDController::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
    {
        auto posComp_1 = _ecm.Component<components::JointPosition>(joint_1);
        auto posComp_2 = _ecm.Component<components::JointPosition>(joint_2);

        double theta_1 = posComp_1->Data()[0];
        double theta_2 = -posComp_2->Data()[0];

        if(_info.paused) { return; }

        auto forceComp_1 = _ecm.Component<components::JointForceCmd>(joint_1);
        auto forceComp_2 = _ecm.Component<components::JointForceCmd>(joint_2);


        forceComp_1->Data()[0] = pid_1.Update(asin(sin(theta_1-M_PI)), _info.dt);
        forceComp_2->Data()[0] = pid_2.Update(asin(sin(theta_2-M_PI)), _info.dt);

        std::cout << "=======================================\n";
        std::cout << posComp_1->Data()[0] << "\n";
        std::cout << posComp_2->Data()[0] << "\n";

        std::cout << forceComp_1->Data()[0] << "\n";
        std::cout << forceComp_2->Data()[0] << "\n";
    }
}