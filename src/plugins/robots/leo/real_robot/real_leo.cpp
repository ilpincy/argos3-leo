#include "real_leo.h"
#include "real_leo_posetwist_actuator.h"
#include "real_leo_odometry_sensor.h"
#include "real_leo_ar_tag_sensor.h"
#include "real_leo_wifi_actuator.h"
#include "real_leo_wifi_sensor.h"

#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

#ifdef catkin_FOUND
CRealLeo::CRealLeo() :
   m_pcNodeHandle(nullptr) {
}
#else
CRealLeo::CRealLeo() {
}
#endif // catkin_FOUND

/****************************************/
/****************************************/

CRealLeo::~CRealLeo() {
}

/****************************************/
/****************************************/

void CRealLeo::InitRobot() {
   /* Initialize this robot as a node */
   // TODO: extract robot name from hostname
#ifdef catkin_FOUND
   int argc = 0;
   ros::init(argc, nullptr, "leo");
   m_pcNodeHandle = new ros::NodeHandle();
#endif // catkin_FOUND
}

/****************************************/
/****************************************/

void CRealLeo::Destroy() {
   for(std::vector<CRealLeoDevice*>::iterator it = m_vecActuators.begin();
       it != m_vecActuators.end();
       ++it) {
      dynamic_cast<CCI_Actuator*>(*it)->Destroy();
   }
   for(std::vector<CRealLeoDevice*>::iterator it = m_vecSensors.begin();
       it != m_vecActuators.end();
       ++it) {
      dynamic_cast<CCI_Sensor*>(*it)->Destroy();
   }
#ifdef catkin_FOUND
   /* Close ROS node */
   ros::shutdown();
#endif // catkin_FOUND
}

/****************************************/
/****************************************/

CCI_Actuator* CRealLeo::MakeActuator(const std::string& str_name) {
   if(str_name == "leo_posetwist") {
#ifdef catkin_FOUND
      CRealLeoPoseTwistActuator* pcAct = new CRealLeoPoseTwistActuator(*m_pcNodeHandle);
#else
      CRealLeoPoseTwistActuator* pcAct = new CRealLeoPoseTwistActuator();
#endif // catkin_FOUND
      m_vecActuators.push_back(pcAct);
      LOG << "[INFO] Successfully initialized actuator \"" << str_name << std::endl;
      return pcAct;
   }
   if(str_name == "leo_wifi") {
      CRealLeoWiFiActuator* pcActuator = new CRealLeoWiFiActuator();
      m_vecActuators.push_back(pcActuator);
      LOG << "[INFO] Successfully initialized actuator \"" << str_name << std::endl;
      return pcActuator;
   }
   return nullptr;
}

/****************************************/
/****************************************/

CCI_Sensor* CRealLeo::MakeSensor(const std::string& str_name) {
   if(str_name == "leo_odometry") {
#ifdef catkin_FOUND
      CRealLeoOdometrySensor* pcSensor = new CRealLeoOdometrySensor(*m_pcNodeHandle);
#else
      CRealLeoOdometrySensor* pcSensor = new CRealLeoOdometrySensor();
#endif // catkin_FOUND
      m_vecSensors.push_back(pcSensor);
      LOG << "[INFO] Successfully initialized sensor \"" << str_name << std::endl;
      return pcSensor;
   }
   if(str_name == "leo_wifi") {
      CRealLeoWiFiSensor* pcSensor = new CRealLeoWiFiSensor();
      m_vecSensors.push_back(pcSensor);
      LOG << "[INFO] Successfully initialized sensor \"" << str_name << std::endl;
      return pcSensor;
   }
   else if(str_name == "leo_ar_tag") {
      #ifdef catkin_FOUND
         CRealLeoArTagSensor* pcSensor = new CRealLeoArTagSensor(*m_pcNodeHandle);
      #else
         CRealLeoArTagSensor* pcSensor = new CRealLeoArTagSensor();
      #endif // catkin_FOUND
      m_vecSensors.push_back(pcSensor);
      LOG << "[INFO] Successfully initialized sensor \"" << str_name << std::endl;
      return pcSensor;
   }
   return nullptr;
}

/****************************************/
/****************************************/

void CRealLeo::Sense(Real f_elapsed_time) {
#ifdef catkin_FOUND
   /* Tell ROS to call message handlers */
   ros::spinOnce();
#endif // catkin_FOUND
   LOG.Flush();
   LOGERR.Flush();
}

/****************************************/
/****************************************/

void CRealLeo::Control() {
   CRealRobot::Control();
   LOG.Flush();
   LOGERR.Flush();
}

/****************************************/
/****************************************/

void CRealLeo::Act(Real f_elapsed_time) {
   /* Go through actuators and let them do their thing */
   for(size_t i = 0; i < m_vecActuators.size(); ++i) {
      m_vecActuators[i]->Do(f_elapsed_time);
   }
#ifdef catkin_FOUND
   /* Tell ROS to publish the messages */
   ros::spinOnce();
#endif // catkin_FOUND
   LOG.Flush();
   LOGERR.Flush();
}

/****************************************/
/****************************************/

