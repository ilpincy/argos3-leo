#ifndef REAL_LEO_LOCALIZATION_SENSOR_H // Changed include guard name to be more general
#define REAL_LEO_LOCALIZATION_SENSOR_H

#include <argos3/plugins/robots/leo/control_interface/ci_leo_localization_sensor.h>
#include <argos3/plugins/robots/leo/real_robot/real_leo_ros_device.h>
#ifdef catkin_FOUND
#include <tf2_ros/transform_listener.h> // Include tf2 headers
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For converting to RPY
#include <geometry_msgs/Pose.h> // Include for Pose
#include <ros/ros.h>
#include <ros/timer.h> // Add this include

#endif // catkin_FOUND

using namespace argos;

class CRealLeoLocalizationSensor :
   public CCI_LeoLocalizationSensor,
   public CRealLeoROSDevice {

public:

   #ifdef catkin_FOUND
      CRealLeoLocalizationSensor(ros::NodeHandle& c_node_handle, std::string str_robot_frame, std::string str_map_frame); // Added frame names as parameters
   #endif // catkin_FOUND

   virtual ~CRealLeoLocalizationSensor() {}

   virtual void Do(Real f_elapsed_time) {}
   void Update(const ros::TimerEvent& event);

    geometry_msgs::TransformStamped m_lastTransformStamped;
    bool m_bLastTransformValid;

   #ifdef catkin_FOUND

      tf2_ros::Buffer m_tfBuffer;
      tf2_ros::TransformListener m_tfListener;

    //   geometry_msgs::Pose m_ros_pose; // Store the ROS Pose
    //   CCI_LeoLocalizationSensor::SReading m_cReading; // ARGoS sensor reading
      std::string m_strRobotFrame; // Frame for the robot (e.g., "base_link", "base_footprint")
      std::string m_strMapFrame;    // Frame for the map  (e.g., "map", "odom")

      ros::Timer m_updateTimer; // Add timer member

   #endif // catkin_FOUND
};

#endif // REAL_LEO_LOCALIZATION_SENSOR_H