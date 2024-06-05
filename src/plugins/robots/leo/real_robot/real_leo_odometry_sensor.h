#ifndef REAL_LEO_ODOMETRY_SENSOR_H
#define REAL_LEO_ODOMETRY_SENSOR_H

#include <argos3/plugins/robots/leo/control_interface/ci_leo_odometry_sensor.h>
#include <argos3/plugins/robots/leo/real_robot/real_leo_device.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

using namespace argos;

class CRealLeoOdometrySensor :
   public CCI_LeoOdometrySensor,
   public CRealLeoDevice {

public:

   CRealLeoOdometrySensor(ros::NodeHandle& c_node_handle);
   
   virtual ~CRealLeoOdometrySensor() {}

   void Update(const nav_msgs::Odometry& message);

   ros::Subscriber c_sub;

};

#endif // REAL_LEO_ODOMETRY_SENSOR_H
