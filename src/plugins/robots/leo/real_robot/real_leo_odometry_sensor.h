#ifndef REAL_LEO_ODOMETRY_SENSOR_H
#define REAL_LEO_ODOMETRY_SENSOR_H

#include <argos3/plugins/robots/leo/control_interface/ci_leo_odometry_sensor.h>
#include <argos3/plugins/robots/leo/real_robot/real_leo_device.h>
#ifdef catkin_FOUND
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#endif // catkin_FOUND

using namespace argos;

class CRealLeoOdometrySensor :
   public CCI_LeoOdometrySensor,
   public CRealLeoDevice {

public:

#ifdef catkin_FOUND
   CRealLeoOdometrySensor(ros::NodeHandle& c_node_handle);
#else
   CRealLeoOdometrySensor();
#endif // catkin_FOUND

   virtual ~CRealLeoOdometrySensor() {}

#ifdef catkin_FOUND
   void Update(const nav_msgs::Odometry& message);
#endif // catkin_FOUND

#ifdef catkin_FOUND
   ros::Subscriber c_sub;
#endif // catkin_FOUND

};

#endif // REAL_LEO_ODOMETRY_SENSOR_H
