#ifndef REAL_LEO_POSETWIST_ACTUATOR_H
#define REAL_LEO_POSETWIST_ACTUATOR_H

#include <argos3/plugins/robots/leo/control_interface/ci_leo_posetwist_actuator.h>
#include <argos3/plugins/robots/leo/real_robot/real_leo_ros_device.h>
#include <argos3/core/utility/math/angles.h>
#ifdef catkin_FOUND
#include <ros/ros.h>
#endif // catkin_FOUND

using namespace argos;

class CRealLeoPoseTwistActuator :
   public CCI_LeoPoseTwistActuator,
   public CRealLeoROSDevice {

public:

#ifdef catkin_FOUND
   CRealLeoPoseTwistActuator(ros::NodeHandle& c_node_handle);
#else   
   CRealLeoPoseTwistActuator() {}
#endif // catkin_FOUND
   
   virtual ~CRealLeoPoseTwistActuator() {}

   #ifdef catkin_FOUND
     virtual void Do(Real f_elapsed_time);
   #else   
     virtual void Do(Real f_elapsed_time) {}
   #endif // catkin_FOUND
   
   virtual void SetLinearVelocity(Real f_velocity){
     // TODO: ARGoS works with cm/s, what does the Leo do?
     m_fDesiredLinearVelocity = f_velocity;
   }
   
     virtual void SetAngularVelocity(const CRadians& c_velocity) {
     m_cDesiredAngularVelocity = c_velocity;
   }

private:

#ifdef catkin_FOUND
   ros::Publisher m_cCmdVelPub;
#endif // catkin_FOUND
   Real m_fDesiredLinearVelocity;
   CRadians m_cDesiredAngularVelocity;
};

#endif
