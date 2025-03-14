#include "real_leo_navigation_actuator.h"
#include <argos3/core/utility/logging/argos_log.h>
#ifdef catkin_FOUND
#include <geometry_msgs/Twist.h> // You might not need this anymore, remove if unused
// #include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For quaternion conversion
#endif // catkin_FOUND

/****************************************/
/****************************************/

CRealLeoNavigationActuator::CRealLeoNavigationActuator(ros::NodeHandle& c_node_handle) :
   CRealLeoROSDevice(c_node_handle),
   m_cMoveBaseClient(GetNodeHandle(), "move_base", true) // Initialize action client, true for spin thread
{
   LOG << "[RealLeoNavigationActuator] Connecting to move_base action server..." << std::endl;
   m_cMoveBaseClient.waitForServer(ros::Duration(1.0)); // Wait for server to start (with timeout)
   if(!m_cMoveBaseClient.waitForServer(ros::Duration(1.0))){
     LOGERR << "[RealLeoNavigationActuator] Could not connect to move_base action server after timeout!" << std::endl;
     // Consider throwing an exception or handling the error appropriately
   } else {
     LOG << "[RealLeoNavigationActuator] Connected to move_base action server." << std::endl;
   }
}

/****************************************/
/****************************************/

void CRealLeoNavigationActuator::Do(Real f_elapsed_time) {
   #ifdef catkin_FOUND
   if(m_cGoalPose.header.frame_id != "") { // Check if a goal pose is set (frame_id as flag)
     m_cGoalPose.header.stamp = ros::Time::now();
     move_base_msgs::MoveBaseGoal goal;
     goal.target_pose = m_cGoalPose;

    //  LOG << "[RealLeoNavigationActuator] Sending navigation goal: x=" << goal.target_pose.pose.position.x
    //      << ", y=" << goal.target_pose.pose.position.y
    //      << ", theta=" << tf2::getYaw(goal.target_pose.pose.orientation) << std::endl;

     m_cMoveBaseClient.sendGoal(goal);
     m_cGoalPose.header.frame_id = ""; // Clear the goal after sending (send goal only once per SetNavigationGoal call)
   }
   #endif
}

void CRealLeoNavigationActuator::setGoal(const argos::CVector3 &cGoalPosition) {
  m_cGoalPose.header.frame_id = "map";
  m_cGoalPose.pose.position.x = cGoalPosition.GetX();
  m_cGoalPose.pose.position.y = cGoalPosition.GetY();
  m_cGoalPose.pose.orientation.w = 1;
}

void CRealLeoNavigationActuator::setGoal(
    const argos::CVector3 &cGoalPosition,
    const argos::CQuaternion &cGoalOrientation) {
  m_cGoalPose.header.frame_id = "map";
  m_cGoalPose.pose.position.x = cGoalPosition.GetX();
  m_cGoalPose.pose.position.y = cGoalPosition.GetY();

  m_cGoalPose.pose.orientation.x = cGoalOrientation.GetX();
  m_cGoalPose.pose.orientation.y = cGoalOrientation.GetY();
  m_cGoalPose.pose.orientation.z = cGoalOrientation.GetZ();
  m_cGoalPose.pose.orientation.w = cGoalOrientation.GetW();
}
