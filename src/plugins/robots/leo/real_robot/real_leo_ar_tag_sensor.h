#ifndef REAL_LEO_AR_TAG_SENSOR_H
#define REAL_LEO_AR_TAG_SENSOR_H

#include <argos3/plugins/robots/leo/control_interface/ci_leo_ar_tag_sensor.h>
#include <argos3/plugins/robots/leo/real_robot/real_leo_ros_device.h>
#ifdef catkin_FOUND
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ros/ros.h>
#endif // catkin_FOUND

using namespace argos;

class CRealLeoArTagSensor :
   public CCI_LeoArTagSensor,
   public CRealLeoROSDevice {

public:

   #ifdef catkin_FOUND
      CRealLeoArTagSensor(ros::NodeHandle& c_node_handle);
   #else
      CRealLeoArTagSensor() {}
   #endif // catkin_FOUND
   
   virtual ~CRealLeoArTagSensor() {}

   virtual void Do(Real f_elapsed_time) {}


   #ifdef catkin_FOUND
      void Update(const ar_track_alvar_msgs::AlvarMarkers& c_message);
   #endif // catkin_FOUND

   #ifdef catkin_FOUND
      ros::Subscriber m_csub;
   #endif // catkin_FOUND

};

#endif // REAL_LEO_AR_TAG_SENSOR_H
