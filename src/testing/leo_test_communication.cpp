/* Include the controller definition */
#include "leo_test_communication.h"
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

CLeoTestCommunication::CLeoTestCommunication() {}

/****************************************/
/****************************************/

void CLeoTestCommunication::Init(TConfigurationNode& t_node) {

   m_pcWiFiActuator = GetActuator<CCI_LeoWiFiActuator>("leo_wifi");
   m_pcWiFiSensor   = GetSensor  <CCI_LeoWiFiSensor>("leo_wifi");
}

/****************************************/
/****************************************/

void CLeoTestCommunication::ControlStep() {
   /* Send message */
   static UInt32 unCounter = 0;
   if(unCounter % 100 == 0) {
      CByteArray cBuf;
      cBuf << unCounter << GetId();
      m_pcWiFiActuator->SendToMany(cBuf);
      // RLOG << "[" << unCounter << "] Sent " << GetId() << std::endl;
      /* List received messages */
      std::vector<CCI_LeoWiFiSensor::SMessage> vecMessages;
      m_pcWiFiSensor->GetMessages(vecMessages);
      if(vecMessages.empty()) {
         RLOG << "No messages received" << std::endl;
      }
      else {
         for(std::vector<CCI_LeoWiFiSensor::SMessage>::iterator itM = vecMessages.begin();
             itM != vecMessages.end();
             ++itM) {
            UInt32 unRCounter;
            std::string strRId;
            itM->Payload >> unRCounter;
            itM->Payload >> strRId;
            RLOG << "Received message #" << unRCounter << " from " << strRId << std::endl;
         }
      }
   }
   /* Increase counter */
   ++unCounter;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CLeoTestCommunication, "leo_test_communication")
