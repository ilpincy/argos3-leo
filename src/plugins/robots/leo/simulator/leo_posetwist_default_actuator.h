/**
 * @file <argos3/plugins/robots/leo/simulator/leo_posetwist_default_actuator.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef LEO_POSETWIST_ACTUATOR_DEFAULT_H
#define LEO_POSETWIST_ACTUATOR_DEFAULT_H

#include <string>
#include <map>

namespace argos {
   class CLeoPoseTwistDefaultActuator;
}

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/actuator.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_posetwist_actuator.h>
#include <argos3/plugins/robots/leo/simulator/leo_entity.h>

namespace argos {

   class CLeoPoseTwistDefaultActuator : public CSimulatedActuator,
                                        public CCI_LeoPoseTwistActuator {

   public:

      CLeoPoseTwistDefaultActuator();
      virtual ~CLeoPoseTwistDefaultActuator() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void SetLinearVelocity(Real f_velocity);
      virtual void SetAngularVelocity(const CRadians& c_velocity);

      virtual void Update();
      virtual void Reset();

   protected:

      CLeoEntity* m_pcLeoEntity;
      CEmbodiedEntity* m_pcEmbodiedEntity;
      Real m_fDesiredLinearVelocity;
      CRadians m_cDesiredAngularVelocity;
      
   };

}

#endif
