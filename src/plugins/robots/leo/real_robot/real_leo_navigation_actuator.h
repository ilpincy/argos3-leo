#ifndef REAL_LEO_NAVIGATION_ACTUATOR_H
#define REAL_LEO_NAVIGATION_ACTUATOR_H

#include <argos3/plugins/robots/leo/control_interface/ci_leo_navigation_actuator.h> // Changed from PoseTwist to NavigationActuator
#include <argos3/plugins/robots/leo/real_robot/real_leo_ros_device.h>
#include <argos3/core/utility/math/angles.h>
#ifdef catkin_FOUND
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h> // Required for Action Client
#include <move_base_msgs/MoveBaseAction.h>       // Message type for move_base action
#include <geometry_msgs/PoseStamped.h>           // To create Pose goals
#endif // catkin_FOUND

using namespace argos;

class CRealLeoNavigationActuator :
   public CCI_LeoNavigationActuator, // Now using CCI_LeoNavigationActuator
   public CRealLeoROSDevice {

public:

#ifdef catkin_FOUND
   CRealLeoNavigationActuator(ros::NodeHandle& c_node_handle);
#else
   CRealLeoNavigationActuator() {}
#endif // catkin_FOUND

   virtual ~CRealLeoNavigationActuator() {}

   #ifdef catkin_FOUND
     virtual void Do(Real f_elapsed_time);
   #else
     virtual void Do(Real f_elapsed_time) {}
   #endif // catkin_FOUND


    virtual void setGoal(geometry_msgs::PoseStamped& cGoalPose) override{
        m_cGoalPose = cGoalPose;
    }

    virtual void setGoal(const argos::CVector3 &cGoalPosition) override;

    virtual void setGoal(const argos::CVector3 &cGoalPosition,
                         const argos::CQuaternion &cGoalOrientation) override;

    virtual void cancelGoal() {
      m_cMoveBaseClient.cancelGoal();
    }

private:

// #ifdef catkin_FOUND
   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> m_cMoveBaseClient; // Action Client for move_base
   geometry_msgs::PoseStamped m_cGoalPose; // Store the desired goal pose
// #endif // catkin_FOUND

};

#endif