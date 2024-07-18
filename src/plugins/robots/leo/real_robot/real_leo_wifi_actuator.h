#ifndef REAL_LEO_WIFI_ACTUATOR_H
#define REAL_LEO_WIFI_ACTUATOR_H

#include <argos3/plugins/robots/leo/control_interface/ci_leo_wifi_actuator.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_wifi_sensor.h>
#include <argos3/plugins/robots/leo/real_robot/real_leo_device.h>
#include <netinet/in.h>

using namespace argos;

class CRealLeoWiFiActuator : public CCI_LeoWiFiActuator,
                             public CRealLeoDevice {

public:

   virtual ~CRealLeoWiFiActuator() {}

   virtual void Init(TConfigurationNode& t_node);

   virtual void Destroy();

   virtual void Do(Real f_elapsed_time);

   virtual void SendToOne(const std::string& str_addr,
                          const CByteArray& c_message);
   
   virtual void SendToMany(const CByteArray& c_message);

   virtual void SendToAll(const CByteArray& c_payload);
   
protected:

   int m_nMulticastSocket;
   struct sockaddr_in m_tMulticastAddr;

   std::vector<CCI_LeoWiFiSensor::SMessage> m_vecMsgQueue;

};

#endif // REAL_LEO_WIFI_ACTUATOR_H
