/* Include the controller definition */
#include "leo_swarm_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <iostream>

CLeoSwarmController::CLeoSwarmController() {}

void CLeoSwarmController::Init(TConfigurationNode& t_node) {

   leoPoseTwistActuator = GetActuator<CCI_LeoPoseTwistActuator>("leo_posetwist");
   leoOdometrySensor = GetSensor<CCI_LeoOdometrySensor>("leo_odometry");
   leoArTagSensor = GetSensor<CCI_LeoArTagSensor>("leo_ar_tag");
   m_pcWiFiActuator = GetActuator<CCI_LeoWiFiActuator>("leo_wifi");
   m_pcWiFiSensor = GetSensor<CCI_LeoWiFiSensor>("leo_wifi");

   holds[GetId()] = 0;
}

void CLeoSwarmController::ControlStep() {

    // Update hold states from received messages

    std::vector<CCI_LeoWiFiSensor::SMessage> vecMessages;
    m_pcWiFiSensor->GetMessages(vecMessages);

    if(vecMessages.empty()) {
        std::cout << "No messages received" << std::endl;
    }
    else {
        for(std::vector<CCI_LeoWiFiSensor::SMessage>::iterator itM = vecMessages.begin();
            itM != vecMessages.end();
            ++itM) {
        UInt32 holdRequest;
        std::string strRId;
        itM->Payload >> holdRequest;
        itM->Payload >> strRId;

        if (strRId == GetId()) {
            continue;
        }

        holds[strRId] = holdRequest;
        }
    }

    // Do I see anything?
    int detectedTag = leoArTagSensor->GetReadings().size() > 0;

    // Update others if if different to last recorded
    if (holds[GetId()] != detectedTag) {
        CByteArray cBuf;
        cBuf << detectedTag << GetId();
        m_pcWiFiActuator->SendToMany(cBuf);
    }

    // Update local hold state
    holds[GetId()] = detectedTag;

    // Determine whether to hold
    bool activateHold = false;

    for (const auto& [key, value] : holds) {
        if(value != 0) {
            activateHold = true;
            break;
        }
    }

    if (activateHold)
        leoPoseTwistActuator->SetLinearVelocity(0.0);
    else
        leoPoseTwistActuator->SetLinearVelocity(0.05);

}

REGISTER_CONTROLLER(CLeoSwarmController, "leo_swarm_controller")
