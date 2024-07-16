#ifndef REAL_LEO_POSETWIST_ACTUATOR_H
#define REAL_LEO_POSETWIST_ACTUATOR_H

#include <argos3/plugins/robots/leo/control_interface/ci_leo_posetwist_actuator.h>
#include <argos3/plugins/robots/leo/real_robot/real_leo_device.h>
#include <argos3/core/utility/math/angles.h>
#include <ros/ros.h>

using namespace argos;

class CRealLeoPoseTwistActuator :
   public CCI_LeoPoseTwistActuator,
   public CRealLeoDevice {

public:

   CRealLeoPoseTwistActuator(ros::NodeHandle& c_node_handle);
   
   virtual ~CRealLeoPoseTwistActuator();

   virtual void Do(Real f_elapsed_time);
   
   virtual void SetLinearVelocity(Real f_velocity);

   virtual void SetAngularVelocity(const CRadians& c_velocity);

private:

   ros::Publisher m_cCmdVelPub;
   Real m_fDesiredLinearVelocity;
   CRadians m_cDesiredAngularVelocity;
};

#endif
