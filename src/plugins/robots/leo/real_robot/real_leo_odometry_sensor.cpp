#include "real_leo_odometry_sensor.h"
#include <argos3/core/utility/math/general.h>

/****************************************/
/****************************************/

CRealLeoOdometrySensor::CRealLeoOdometrySensor(ros::NodeHandle& c_node_handle) :
   CRealLeoDevice(c_node_handle) {
   c_sub = c_node_handle.subscribe("/odometry_merged",
                              1,
                              &CRealLeoOdometrySensor::Update,
                              this);
}

/****************************************/
/****************************************/

void CRealLeoOdometrySensor::Update(const nav_msgs::Odometry& message) {
   // TODO fill into m_sReading
   geometry_msgs::Point cPosition = message.pose.pose.position;
   geometry_msgs::Quaternion cOrientation = message.pose.pose.orientation;
   m_sReading.Position.Set(cPosition.x, cPosition.y, cPosition.z);
   m_sReading.Orientation.Set(cOrientation.x, cOrientation.y, cOrientation.z, cOrientation.w);
}

/****************************************/
/****************************************/