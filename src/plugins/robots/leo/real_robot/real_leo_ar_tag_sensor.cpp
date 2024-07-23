#include "real_leo_ar_tag_sensor.h"
#include <argos3/core/utility/math/general.h>

/****************************************/
/****************************************/

CRealLeoArTagSensor::CRealLeoArTagSensor(ros::NodeHandle& c_node_handle) :
   CRealLeoROSDevice(c_node_handle) {
   m_csub = c_node_handle.subscribe("/ar_pose_marker",
                              1,
                              &CRealLeoArTagSensor::Update,
                              this);
}

/****************************************/
/****************************************/

void CRealLeoArTagSensor::Update(const ar_track_alvar_msgs::AlvarMarkers& c_message) {
  m_tReadings.clear();
  for (const auto& cSingleMarker : c_message.markers) {
    SReading cMarker;
    cMarker.TagId = cSingleMarker.id;
    cMarker.Confidence = cSingleMarker.confidence;
    cMarker.Position.Set(cSingleMarker.pose.pose.position.x, cSingleMarker.pose.pose.position.y, cSingleMarker.pose.pose.position.z);
    cMarker.Orientation.Set(cSingleMarker.pose.pose.orientation.x, cSingleMarker.pose.pose.orientation.y, cSingleMarker.pose.pose.orientation.z, cSingleMarker.pose.pose.orientation.w);
    m_tReadings.push_back(cMarker);
  }
}

/****************************************/
/****************************************/