#ifndef REAL_LEO_AR_TAG_SENSOR_H
#define REAL_LEO_AR_TAG_SENSOR_H

#include <argos3/plugins/robots/leo/control_interface/ci_leo_ar_tag_sensor.h>
#include <argos3/plugins/robots/leo/real_robot/real_leo_device.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ros/ros.h>

using namespace argos;

class CRealLeoArTagSensor :
   public CCI_LeoArTagSensor,
   public CRealLeoDevice {

public:

   CRealLeoArTagSensor(ros::NodeHandle& c_node_handle);
   
   virtual ~CRealLeoArTagSensor() {}

   void Update(const ar_track_alvar_msgs::AlvarMarkers& message);

   ros::Subscriber c_sub;

};

#endif // REAL_LEO_AR_TAG_SENSOR_H
