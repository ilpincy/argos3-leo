#include "real_leo_posetwist_actuator.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <geometry_msgs/Twist.h>

/****************************************/
/****************************************/

CRealLeoPoseTwistActuator::CRealLeoPoseTwistActuator(ros::NodeHandle& c_node_handle) :
   CRealLeoDevice(c_node_handle) {
   m_cCmdVelPub = GetNodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}
   
/****************************************/
/****************************************/

CRealLeoPoseTwistActuator::~CRealLeoPoseTwistActuator() {
}
   
/****************************************/
/****************************************/

void CRealLeoPoseTwistActuator::Do(Real f_elapsed_time) {
   /* Create a Twist message to send to ROS */
   geometry_msgs::Twist cVelMsg;
   /* Set linear velocity */
   cVelMsg.linear.x = m_fDesiredLinearVelocity;
   cVelMsg.linear.y = 0.0;
   cVelMsg.linear.z = 0.0;
   /* Set angular velocity */
   cVelMsg.angular.x = 0.0;
   cVelMsg.angular.y = 0.0;
   cVelMsg.angular.z = m_cDesiredAngularVelocity.GetValue();
   /* Publish the message */
   m_cCmdVelPub.publish(cVelMsg);
}

/****************************************/
/****************************************/

void CRealLeoPoseTwistActuator::SetLinearVelocity(Real f_velocity) {
   // TODO: ARGoS works with cm/s, what does the Leo do?
   m_fDesiredLinearVelocity = f_velocity;
}

/****************************************/
/****************************************/

void CRealLeoPoseTwistActuator::SetAngularVelocity(const CRadians& c_velocity) {
   m_cDesiredAngularVelocity = c_velocity;
}

/****************************************/
/****************************************/
