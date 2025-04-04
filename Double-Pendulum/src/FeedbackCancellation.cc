
#include <gz/sim/System.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/PID.hh>


#include "FeedbackCancellation.hh"

using namespace gz::sim;
using namespace std;

namespace dbl_pend_ctrl {
    void FeedbackCancellation::Configure(const Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        EntityComponentManager &_ecm,
                        EventManager &/*_eventMgr*/) 
    {
        std::string joint_1_name;
        std::string joint_2_name;

        if(_sdf->HasElement("joint_1_name")){ joint_1_name = _sdf->Get<std::string>("joint_1_name"); }
        if(_sdf->HasElement("joint_2_name")){ joint_2_name = _sdf->Get<std::string>("joint_2_name"); }

        if(_sdf->HasElement("mass_1")){ m_1 = _sdf->Get<double>("mass_1"); }
        if(_sdf->HasElement("mass_2")){ m_2 = _sdf->Get<double>("mass_2"); }

        if(_sdf->HasElement("length_1")){ l_1 = _sdf->Get<double>("length_1"); }
        if(_sdf->HasElement("length_2")){ l_2 = _sdf->Get<double>("length_2"); }

        if(_sdf->HasElement("theta_1_0")){ theta_1_0 = _sdf->Get<double>("theta_1_0"); }
        if(_sdf->HasElement("theta_2_0")){ theta_2_0 = _sdf->Get<double>("theta_2_0"); }


        joint_1 = _ecm.EntityByComponents(components::Name(joint_1_name));
        joint_2 = _ecm.EntityByComponents(components::Name(joint_2_name));

        _ecm.CreateComponent(joint_1, components::JointPosition({0}));
        _ecm.CreateComponent(joint_1, components::JointForceCmd({0}));
        _ecm.CreateComponent(joint_1, components::JointPositionReset({theta_1_0}));

        _ecm.CreateComponent(joint_2, components::JointPosition({0}));
        _ecm.CreateComponent(joint_2, components::JointForceCmd({0}));
        _ecm.CreateComponent(joint_2, components::JointPositionReset({theta_2_0}));

    }

    
    void FeedbackCancellation::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
    {
        auto posComp_1 = _ecm.Component<components::JointPosition>(joint_1);
        auto posComp_2 = _ecm.Component<components::JointPosition>(joint_2);

        double theta_1 = posComp_1->Data()[0];
        double theta_2 = -posComp_2->Data()[0];

        if(_info.paused) { return; }

        auto forceComp_1 = _ecm.Component<components::JointForceCmd>(joint_1);
        auto forceComp_2 = _ecm.Component<components::JointForceCmd>(joint_2);

        forceComp_1->Data()[0] = 2 * (m_1 + m_2) * g * l_1 * sin(theta_1);
        forceComp_2->Data()[0] = 2 * (m_2) * g * l_2 * sin(theta_2);

        std::cout << "=======================================\n";
        std::cout << posComp_1->Data()[0] << "\n";
        std::cout << posComp_2->Data()[0] << "\n";

        std::cout << forceComp_1->Data()[0] << "\n";
        std::cout << forceComp_2->Data()[0] << "\n";
    }
}