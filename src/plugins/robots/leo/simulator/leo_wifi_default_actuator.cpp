#include "leo_wifi_default_actuator.h"
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/simulator.h>

namespace argos {

/****************************************/
/****************************************/

void CLeoWiFiActuator::SetRobot(CComposableEntity& c_entity) {
   m_pcWiFiEquippedEntity = &c_entity.GetComponent<CWiFiEquippedEntity>("wifiEntity");
   m_pcWiFiEquippedEntity->Enable();
}

/****************************************/
/****************************************/

void CLeoWiFiActuator::Update() {
    for(auto& e : m_vecMsgQueue) {
        m_pcWiFiEquippedEntity->AppendData(e);
    }
    m_vecMsgQueue.clear();
}

/****************************************/
/****************************************/

void CLeoWiFiActuator::Reset() {
    m_vecMsgQueue.clear();
}

/****************************************/
/****************************************/

void CLeoWiFiActuator::SendToMany(const CByteArray& c_message) {
   std::cout << __FILE__ << " " << __func__ << std::endl;
   m_vecMsgQueue.push_back({"", c_message});
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
