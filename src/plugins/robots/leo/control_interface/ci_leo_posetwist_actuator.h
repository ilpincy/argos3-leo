#ifndef CI_LEO_POSETWIST_ACTUATOR_H
#define CI_LEO_POSETWIST_ACTUATOR_H

/* To avoid dependency problems when including */
namespace argos {
   class CCI_LeoPoseTwistActuator;
}

#include <argos3/core/control_interface/ci_actuator.h>
#include <argos3/core/utility/math/angles.h>

namespace argos {
   class CCI_LeoPoseTwistActuator : public CCI_Actuator {

   public:

      virtual ~CCI_LeoPoseTwistActuator() {}

      /**
       * Sets the linear velocity of the quadrotor.
       * @param c_velocity The desired linear velocity.
       */
      virtual void SetLinearVelocity(Real f_velocity) = 0;

      /**
       * Sets the rotational velocity of the quadrotor around the local Z axis (yaw).
       * @param c_velocity The desired rotational velocity.
       */
      virtual void SetAngularVelocity(const CRadians& c_speed) = 0;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);
#endif

   };

}

#endif
