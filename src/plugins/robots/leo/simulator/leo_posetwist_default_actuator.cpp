/**
 * @file <argos3/plugins/robots/leo/simulator/leo_posetwist_default_actuator.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "leo_posetwist_default_actuator.h"
#include <argos3/core/utility/logging/argos_log.h>

namespace argos {

   /****************************************/
   /****************************************/

   CLeoPoseTwistDefaultActuator::CLeoPoseTwistDefaultActuator() :
      m_pcLeoEntity(nullptr) {
   }

   /****************************************/
   /****************************************/

   void CLeoPoseTwistDefaultActuator::SetRobot(CComposableEntity& c_entity) {
      m_pcLeoEntity = dynamic_cast<CLeoEntity*>(&c_entity);
      if(!m_pcLeoEntity) {
         THROW_ARGOSEXCEPTION("Error setting leo pose-twist actuator to entity \"" << c_entity.GetId() << "\"");
      }
   }

   /****************************************/
   /****************************************/

   void CLeoPoseTwistDefaultActuator::Init(TConfigurationNode& t_tree) {
      try {
         CCI_LeoPoseTwistActuator::Init(t_tree);
         Reset();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in leo pose-twist actuator.", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CLeoPoseTwistDefaultActuator::SetLinearVelocity(Real f_velocity) {
      m_fDesiredLinearVelocity = f_velocity;
   }

   /****************************************/
   /****************************************/

   void CLeoPoseTwistDefaultActuator::SetAngularVelocity(const CRadians& c_velocity) {
      m_cDesiredAngularVelocity = c_velocity;
   }

   /****************************************/
   /****************************************/

   void CLeoPoseTwistDefaultActuator::Update() {
      m_pcLeoEntity->SetLinearVelocity(m_fDesiredLinearVelocity);
      m_pcLeoEntity->SetAngularVelocity(m_cDesiredAngularVelocity);
   }

   /****************************************/
   /****************************************/

   void CLeoPoseTwistDefaultActuator::Reset() {
      m_fDesiredLinearVelocity = 0.0;
      m_cDesiredAngularVelocity = CRadians::ZERO;
      Update();
   }

   /****************************************/
   /****************************************/

}

REGISTER_ACTUATOR(CLeoPoseTwistDefaultActuator,
                  "leo_posetwist", "default",
                  "Carlo Pinciroli [ilpincy@gmail.com]",
                  "1.0",
                  "The leo posetwist actuator.",
                  "This actuator controls the posetwist of a leo robot. For a\n"
                  "complete description of its usage, refer to the\n"
                  "ci_leo_posetwist_actuator.h file.\n\n"

                  "REQUIRED XML CONFIGURATION\n\n"

                  "  <controllers>\n"
                  "    ...\n"
                  "    <my_controller ...>\n"
                  "      ...\n"
                  "      <actuators>\n"
                  "        ...\n"
                  "        <leo_posetwist implementation=\"default\" />\n"
                  "        ...\n"
                  "      </actuators>\n"
                  "      ...\n"
                  "    </my_controller>\n"
                  "    ...\n"
                  "  </controllers>\n\n"

                  "OPTIONAL XML CONFIGURATION\n\n"

                  "None.\n\n"
                  ,
                  "Usable"
   );
