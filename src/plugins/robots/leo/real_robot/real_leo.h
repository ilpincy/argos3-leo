#ifndef REAL_LEO_H
#define REAL_LEO_H

#include <argos3/core/real_robot/real_robot.h>
#include <argos3/plugins/robots/leo/real_robot/real_leo_device.h>
#ifdef catkin_FOUND
#include <ros/ros.h>
#endif // catkin_FOUND


using namespace argos;

class CRealLeo : public CRealRobot {

public:

   CRealLeo();
   virtual ~CRealLeo();
   virtual void InitRobot();
   virtual void Destroy();
   virtual CCI_Actuator* MakeActuator(const std::string& str_name);
   virtual CCI_Sensor* MakeSensor(const std::string& str_name);
   virtual void Sense(Real f_elapsed_time);
   virtual void Act(Real f_elapsed_time);

private:

#ifdef catkin_FOUND
   ros::NodeHandle* m_pcNodeHandle;
#endif // catkin_FOUND

   std::vector<CRealLeoDevice*> m_vecActuators;
   std::vector<CRealLeoDevice*> m_vecSensors;
};

#endif // REAL_LEO_H
