#ifndef REAL_LEO_DEVICE_H
#define REAL_LEO_DEVICE_H

#include <argos3/core/utility/datatypes/datatypes.h>

#ifdef catkin_FOUND
#include <ros/ros.h>
#endif // catkin_FOUND

using namespace argos;

class CRealLeoDevice {

public:

#ifdef catkin_FOUND
   CRealLeoDevice(ros::NodeHandle& c_node_handle) :
      m_cNodeHandle(c_node_handle) {}
#else
   CRealLeoDevice() {}
#endif // catkin_FOUND

   virtual ~CRealLeoDevice() {}

   virtual void Do(Real f_elapsed_time) {}

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
