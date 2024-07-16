#ifndef REAL_LEO_DEVICE_H
#define REAL_LEO_DEVICE_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <ros/ros.h>

using namespace argos;

class CRealLeoDevice {

public:

   CRealLeoDevice(ros::NodeHandle& c_node_handle) :
      m_cNodeHandle(c_node_handle) {}
   
   virtual ~CRealLeoDevice() {}

   virtual void Do(Real f_elapsed_time) {}

   inline ros::NodeHandle& GetNodeHandle() {
      return m_cNodeHandle;
   }

private:

   ros::NodeHandle& m_cNodeHandle;

};

#endif // REAL_LEO_DEVICE_H
