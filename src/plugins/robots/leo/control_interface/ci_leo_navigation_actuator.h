#ifndef CI_LEO_NAVIGATION_ACTUATOR_H
#define CI_LEO_NAVIGATION_ACTUATOR_H

#include <argos3/core/control_interface/ci_actuator.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
   #ifdef catkin_FOUND
#include <geometry_msgs/PoseStamped.h>           // To create Pose goals
   #endif // catkin_FOUND
namespace argos {

   class CCI_LeoNavigationActuator : public CCI_Actuator {

   public:

      CCI_LeoNavigationActuator() {}
   #ifdef catkin_FOUND
      virtual void setGoal(geometry_msgs::PoseStamped& cGoalPose) = 0;
   #endif // catkin_FOUND
      virtual void setGoal(const argos::CVector3 &cGoalPosition) = 0;

      virtual void setGoal(const argos::CVector3 &cGoalPosition,
                           const argos::CQuaternion &cGoalOrientation) = 0;
      virtual void cancelGoal() = 0;
   };

}
#endif