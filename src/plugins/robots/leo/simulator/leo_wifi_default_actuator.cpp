#include "leo_wifi_default_actuator.h"
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/simulator.h>

namespace argos {

/****************************************/
/****************************************/

void CLeoWiFiActuator::SetRobot(CComposableEntity& c_entity) {
   m_pcRangeAndBearingEquippedEntity = &c_entity.GetComponent<CRABEquippedEntity>("rab");
   m_pcRangeAndBearingEquippedEntity->Enable();
   m_cData.Resize(m_pcRangeAndBearingEquippedEntity->GetMsgSize());
}

/****************************************/
/****************************************/

void CLeoWiFiActuator::Update() {
   m_pcRangeAndBearingEquippedEntity->SetData(m_cData);
}

/****************************************/
/****************************************/

void CLeoWiFiActuator::Reset() {
   m_cData.Zero();
}

/****************************************/
/****************************************/

void CLeoWiFiActuator::SendToMany(const CByteArray& c_message) {
   m_pcRangeAndBearingEquippedEntity->SetData(c_message);
}

/****************************************/
/****************************************/

REGISTER_ACTUATOR(
    CLeoWiFiActuator, "leo_wifi", "default",
    "Davis Catherman [daviscatherman@gmail.com]", "1.0",
    "A simulated wifi actuator for leo.",

    "This Actuator is an empty implementation and does not do anything. In\n"
    "controllers, you must include the ci_leo_wifi_Actuator.h header.\n\n"

    "REQUIRED XML CONFIGURATION\n\n"
    "  <controllers>\n"
    "    ...\n"
    "    <my_controller ...>\n"
    "      ...\n"
    "      <actuators>\n"
    "        ...\n"
    "        <leo_wifi implementation=\"default\" />\n"
    "        ...\n"
    "      </actuators>\n"
    "      ...\n"
    "    </my_controller>\n"
    "    ...\n"
    "  </controllers>\n\n"

    "OPTIONAL XML CONFIGURATION\n\n"

    "None.\n",

    "Usable");

} // namespace argos
/****************************************/
/****************************************/
