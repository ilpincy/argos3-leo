/* Include the controller definition */
#include "leo_test_communication.h"
#include <argos3/core/utility/logging/argos_log.h>
#include <chrono>
#include <ctime> 


// using namespace date;
    using namespace std::chrono;

namespace {
double GetTime() {
   auto now = std::chrono::high_resolution_clock::now();
   auto duration = now.time_since_epoch();
   auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration);  
   double decimal_ms = ms.count();
   return decimal_ms;
}
}

/****************************************/
/****************************************/

CLeoTestCommunication::CLeoTestCommunication() {}

/****************************************/
/****************************************/

void CLeoTestCommunication::Init(TConfigurationNode& t_node) {

   m_pcWiFiActuator = GetActuator<CCI_LeoWiFiActuator>("leo_wifi");
   m_pcWiFiSensor   = GetSensor  <CCI_LeoWiFiSensor>("leo_wifi");

   m_initializedTime = GetTime();
   
   m_unCounter = 0;

   m_noMsgPrintTime = 0.0;
}

/****************************************/
/****************************************/

void CLeoTestCommunication::ControlStep() {

   /* Send message */
   if(m_unCounter % 100 == 0) {
      CByteArray cBuf;
      cBuf << m_unCounter << GetId() << GetTime() << m_initializedTime;
      m_pcWiFiActuator->SendToMany(cBuf);
   }

   /* List received messages */
   std::vector<CCI_LeoWiFiSensor::SMessage> vecMessages;
   m_pcWiFiSensor->GetMessages(vecMessages);

   if(GetTime() - m_noMsgPrintTime > 5'000) {
      RLOG << "No messages received" << std::endl;
      m_noMsgPrintTime = GetTime();
   }
   if(vecMessages.size() > 0) {
      for(std::vector<CCI_LeoWiFiSensor::SMessage>::iterator itM = vecMessages.begin();
            itM != vecMessages.end();
            ++itM) {

         // delay = (lRec - rSend) - (rInit - lInit)
         UInt32 unRCounter;
         std::string strRId;

         double rInit;
         double rSend;
         double lRec = GetTime();
         m_noMsgPrintTime = lRec;

         itM->Payload >> unRCounter;
         itM->Payload >> strRId;
         itM->Payload >> rSend;
         itM->Payload >> rInit;
         double delay = (lRec - m_initializedTime) - (rSend - rInit);

         RLOG << GetId() << " :: Received message!" << \
            "\n   #" << unRCounter << \
            "\n   from " << strRId << \
            "\n   at " << lRec-m_initializedTime << \
            "\n   delay " << delay << \
            std::endl;

         if(m_recieveStats.count(strRId) == 0) {
            m_recieveStats[strRId] = {delay, 1};
         } else {
            int c = m_recieveStats[strRId].count;
            double d = m_recieveStats[strRId].delay;
            m_recieveStats[strRId].delay = (d*c+delay) / (c+1);
            m_recieveStats[strRId].count++;
         }

         RLOG << "Avg Stats: " << \
            "\n   From:  " << strRId << \
            "\n   delay: " << m_recieveStats[strRId].delay << \
            "\n   count: " << m_recieveStats[strRId].count << \
            std::endl;
      
      }
   }
   /* Increase counter */
   ++m_unCounter;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CLeoTestCommunication, "leo_test_communication")
