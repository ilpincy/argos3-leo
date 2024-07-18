#include "real_leo_tag_sensor.h"
#include <argos3/core/utility/math/general.h>

/****************************************/
/****************************************/

CRealLeoTagSensor::CRealLeoTagSensor(ros::NodeHandle& c_node_handle) :
   CRealLeoDevice(c_node_handle) {
   c_sub = c_node_handle.subscribe("/ar_pose_marker",
                              1,
                              &CRealLeoTagSensor::Update,
                              this);
}

/****************************************/
/****************************************/

void CRealLeoTagSensor::Update(const ar_track_alvar_msgs::AlvarMarkers& message) {
  m_tReadings.clear();
  for (const auto& single_marker : message.markers) {
    SReading marker;
    marker.TagId = single_marker.id;
    marker.Confidence = single_marker.confidence;
    marker.Position.Set(single_marker.pose.pose.position.x, single_marker.pose.pose.position.y, single_marker.pose.pose.position.z);
    marker.Orientation.Set(single_marker.pose.pose.orientation.x, single_marker.pose.pose.orientation.y, single_marker.pose.pose.orientation.z, single_marker.pose.pose.orientation.w);
    m_tReadings.push_back(marker);
  }
}

/****************************************/
/****************************************/