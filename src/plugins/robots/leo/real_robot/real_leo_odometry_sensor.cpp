#include "real_leo_odometry_sensor.h"
#include <argos3/core/utility/math/general.h>

/****************************************/
/****************************************/

CRealLeoOdometrySensor::CRealLeoOdometrySensor(ros::NodeHandle& c_node_handle) :
   CRealLeoROSDevice(c_node_handle) {
   m_cSub = c_node_handle.subscribe("/odometry_merged_offset",
                                    1,
                                    &CRealLeoOdometrySensor::Update,
                                    this);
}

/****************************************/
/****************************************/

void CRealLeoOdometrySensor::Update(const nav_msgs::Odometry& c_message) {
   // TODO fill into m_sReading
   geometry_msgs::Point cPosition = c_message.pose.pose.position;
   geometry_msgs::Quaternion cOrientation = c_message.pose.pose.orientation;
   m_sReading.Position.Set(cPosition.x, cPosition.y, cPosition.z);
   m_sReading.Orientation.Set(cOrientation.w, cOrientation.x, cOrientation.y, cOrientation.z);
}

/****************************************/
/****************************************/
