#ifndef CI_LEO_NAVIGATION_ACTUATOR_H
#define CI_LEO_NAVIGATION_ACTUATOR_H

#include <argos3/core/control_interface/ci_actuator.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <geometry_msgs/PoseStamped.h>           // To create Pose goals

namespace argos {

   class CCI_LeoNavigationActuator : public CCI_Actuator {

   public:

      CCI_LeoNavigationActuator() {}

      virtual void setGoal(geometry_msgs::PoseStamped& cGoalPose) = 0;

   };

}
#endif