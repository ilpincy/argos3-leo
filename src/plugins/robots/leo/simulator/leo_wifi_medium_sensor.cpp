#include "leo_wifi_medium_sensor.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/plugins/simulator/entities/wifi_equipped_entity.h>
#include <argos3/plugins/simulator/media/wifi_medium.h>

namespace argos {

/****************************************/
/****************************************/

CRange<CRadians> INCLINATION_RANGE(CRadians(0), CRadians(ARGOS_PI));

/****************************************/
/****************************************/

CLeoWiFiMediumSensor::CLeoWiFiMediumSensor() :
    m_pcWiFiEquippedEntity(nullptr),
    m_fDistanceNoiseStdDev(0.0f),
    m_fPacketDropProb(0.0f),
    m_pcRNG(nullptr),
    m_cSpace(CSimulator::GetInstance().GetSpace()),
    m_bShowRays(false) {}

/****************************************/
/****************************************/

void CLeoWiFiMediumSensor::SetRobot(CComposableEntity& c_entity) {
    /* Assign WiFi equipped entity to this sensor */
    m_pcWiFiEquippedEntity = &c_entity.GetComponent<CWiFiEquippedEntity>("wifi");
    /* Get reference to controllable entity */
    m_pcControllableEntity = &c_entity.GetComponent<CControllableEntity>("controller");
}

/****************************************/
/****************************************/

void CLeoWiFiMediumSensor::Init(TConfigurationNode& t_tree) {
    try {
        /* Parent class init */
        CCI_LeoWiFiSensor::Init(t_tree);
        /* Show rays? */
        GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
        /* Parse noise */
        GetNodeAttributeOrDefault(t_tree, "noise_std_dev", m_fDistanceNoiseStdDev, m_fDistanceNoiseStdDev);
        GetNodeAttributeOrDefault(t_tree, "packet_drop_prob", m_fPacketDropProb, m_fPacketDropProb);
        if((m_fPacketDropProb > 0.0f) ||
        (m_fDistanceNoiseStdDev > 0.0f)) {
        m_pcRNG = CRandom::CreateRNG("argos");
        }
        /* Get WiFi medium from id specified in the XML */
        std::string strMedium;
        GetNodeAttribute(t_tree, "medium", strMedium);
        m_pcWiFiMedium = &(CSimulator::GetInstance().GetMedium<CWiFiMedium>(strMedium));
        /* Assign WiFi entity to the medium */
        m_pcWiFiEquippedEntity->SetMedium(*m_pcWiFiMedium);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing the range and bearing medium sensor", ex);
    }
    /* sensor is enabled by default */
    Enable();
}

/****************************************/
/****************************************/

void CLeoWiFiMediumSensor::Update() {
    /* sensor is disabled--nothing to do */
    if (IsDisabled()) {
    return;
    }
    /** TODO: there's a more efficient way to implement this */
    /* Delete old readings */
    m_vecMsgQueue.clear();
    /* Get list of communicating WiFis */
    const CSet<CWiFiEquippedEntity*,SEntityComparator>& setWiFis = m_pcWiFiMedium->GetWiFisCommunicatingWith(*m_pcWiFiEquippedEntity);

    /* Buffer for the received packet */
    CCI_LeoWiFiSensor::SMessage sMessage;
    /* Go through communicating WiFis and create packets */
    for(CSet<CWiFiEquippedEntity*>::iterator it = setWiFis.begin();
        it != setWiFis.end(); ++it) {
        /* Should we drop this packet? */
        if(m_pcRNG == nullptr || /* No noise to apply */
        !(m_fPacketDropProb > 0.0f &&
            m_pcRNG->Bernoulli(m_fPacketDropProb)) /* Packet is not dropped */
        ) {
        /* Create a reference to the WiFi entity to process */
        CWiFiEquippedEntity& cWiFiEntity = **it;
        /* Add ray if requested */
        if(m_bShowRays) {
            m_pcControllableEntity->AddCheckedRay(false,
                                                    CRay3(cWiFiEntity.GetPosition(),
                                                        m_pcWiFiEquippedEntity->GetPosition()));
        }
        /* Set message data */
        cWiFiEntity.RetrieveData(m_vecMsgQueue);
        
        }
    }
}
    
/****************************************/
/****************************************/

void CLeoWiFiMediumSensor::Reset() {
    // m_sMessage.clear();
}


/****************************************/
/****************************************/
void CLeoWiFiMediumSensor::Enable() {
    m_pcWiFiEquippedEntity->Enable();
    CCI_Sensor::Enable();
}

/****************************************/
/****************************************/

void CLeoWiFiMediumSensor::Disable() {
    m_pcWiFiEquippedEntity->Disable();
    CCI_Sensor::Disable();
}

/****************************************/
/****************************************/

void CLeoWiFiMediumSensor::Destroy() {
    m_pcWiFiMedium->RemoveEntity(*m_pcWiFiEquippedEntity);
}

/****************************************/
/****************************************/

void CLeoWiFiMediumSensor::GetMessages(std::vector<CCI_LeoWiFiSensor::SMessage>& vec_messages) {
//    std::cout << __FILE__ << " " << __func__ << std::endl;
   vec_messages.swap(m_vecMsgQueue);
   m_vecMsgQueue.clear();
}

/****************************************/
/****************************************/

REGISTER_SENSOR(
    CLeoWiFiMediumSensor, "leo_wifi", "default",
    "Davis Catherman [daviscatherman@gmail.com]", "1.0",
    "A simulated wifi sensor for leo.",

    "This sensor is an empty implementation and does not do anything. In\n"
    "controllers, you must include the ci_leo_wifi_sensor.h header.\n\n"

    "REQUIRED XML CONFIGURATION\n\n"
    "  <controllers>\n"
    "    ...\n"
    "    <my_controller ...>\n"
    "      ...\n"
    "      <sensors>\n"
    "        ...\n"
    "        <leo_wifi implementation=\"default\" />\n"
    "        ...\n"
    "      </sensors>\n"
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
