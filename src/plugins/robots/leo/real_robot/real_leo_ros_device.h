#ifndef REAL_LEO_ROS_DEVICE_H
#define REAL_LEO_ROS_DEVICE_H

#include <argos3/plugins/robots/leo/real_robot/real_leo_device.h>

#ifdef catkin_FOUND
#include <ros/ros.h>
#endif // catkin_FOUND

using namespace argos;

class CRealLeoROSDevice : public CRealLeoDevice {

public:

#ifdef catkin_FOUND
   CRealLeoROSDevice(ros::NodeHandle& c_node_handle) :
      m_cNodeHandle(c_node_handle) {}
#else
   CRealLeoROSDevice() {}
#endif // catkin_FOUND

   virtual ~CRealLeoROSDevice() {}

#ifdef catkin_FOUND
   inline ros::NodeHandle& GetNodeHandle() {
      return m_cNodeHandle;
   }
#endif // catkin_FOUND

private:

#ifdef catkin_FOUND
   ros::NodeHandle& m_cNodeHandle;
#endif // catkin_FOUND

};

#endif // REAL_LEO_DEVICE_H
