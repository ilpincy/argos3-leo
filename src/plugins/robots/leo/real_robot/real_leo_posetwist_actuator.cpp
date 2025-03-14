#include "real_leo_posetwist_actuator.h"
#include <argos3/core/utility/logging/argos_log.h>
#ifdef catkin_FOUND
#include <geometry_msgs/Twist.h>
#endif // catkin_FOUND

/****************************************/
/****************************************/

CRealLeoPoseTwistActuator::CRealLeoPoseTwistActuator(ros::NodeHandle& c_node_handle) :
   CRealLeoROSDevice(c_node_handle) {
   m_cCmdVelPub = GetNodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel", 10);
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
