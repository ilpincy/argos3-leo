#include "real_leo.h"
#include "real_leo_posetwist_actuator.h"
#include "real_leo_odometry_sensor.h"
#include "real_leo_ar_tag_sensor.h"
#include <argos3/core/utility/logging/argos_log.h>

CRealLeo::CRealLeo() :
   m_pcNodeHandle(nullptr) {
}

/****************************************/
/****************************************/

CRealLeo::~CRealLeo() {
}

/****************************************/
/****************************************/

void CRealLeo::InitRobot() {
   /* Initialize this robot as a node */
   // TODO: extract robot name from hostname
   int argc = 0;
   ros::init(argc, nullptr, "leo");
   m_pcNodeHandle = new ros::NodeHandle();
}

/****************************************/
/****************************************/

void CRealLeo::Destroy() {
   /* TODO: Stop wheels */
   /* Close ROS node */
   ros::shutdown();
}

/****************************************/
/****************************************/

CCI_Actuator* CRealLeo::MakeActuator(const std::string& str_name) {
   if(str_name == "leo_posetwist") {
      CRealLeoPoseTwistActuator* pcAct = new CRealLeoPoseTwistActuator(*m_pcNodeHandle);
      m_vecActuators.push_back(pcAct);
      LOG << "[INFO] Successfully initialized actuator \"" << str_name << std::endl;
      return pcAct;
   }
   return nullptr;
}

/****************************************/
/****************************************/

CCI_Sensor* CRealLeo::MakeSensor(const std::string& str_name) {
   if(str_name == "leo_odometry") {
      CRealLeoOdometrySensor* pcSensor = new CRealLeoOdometrySensor(*m_pcNodeHandle);
      m_vecSensors.push_back(pcSensor);
      LOG << "[INFO] Successfully initialized sensor \"" << str_name << std::endl;
      return pcSensor;
   }
   else if(str_name == "leo_ar_tag") {
      CRealLeoArTagSensor* pcSensor = new CRealLeoArTagSensor(*m_pcNodeHandle);
      m_vecSensors.push_back(pcSensor);
      LOG << "[INFO] Successfully initialized sensor \"" << str_name << std::endl;
      return pcSensor;
   }
   return nullptr;
}

/****************************************/
/****************************************/

void CRealLeo::Sense(Real f_elapsed_time) {
   /* Tell ROS to call message handlers */
   ros::spinOnce();
}

/****************************************/
/****************************************/

void CRealLeo::Act(Real f_elapsed_time) {
   /* Go through actuators and let them do their thing */
   for(size_t i = 0; i < m_vecActuators.size(); ++i) {
      m_vecActuators[i]->Do(f_elapsed_time);
   }
   /* Tell ROS to publish the messages */
   ros::spinOnce();
}

/****************************************/
/****************************************/

