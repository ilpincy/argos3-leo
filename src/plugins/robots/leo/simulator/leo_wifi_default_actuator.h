#ifndef LEO_WIFI_DEFAULT_ACTUATOR_H
#define LEO_WIFI_DEFAULT_ACTUATOR_H

#include <map>
#include <string>

namespace argos {
class CLeoWiFiActuator;
} // namespace argos

#include <argos3/core/simulator/actuator.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_wifi_actuator.h>
#include <argos3/plugins/simulator/entities/wifi_equipped_entity.h>

namespace argos {
class CLeoWiFiActuator : public CCI_LeoWiFiActuator, public CSimulatedActuator {

public:

  CLeoWiFiActuator() {};
  virtual ~CLeoWiFiActuator() {}

  virtual void SetRobot(CComposableEntity &c_entity);
  virtual void Update();
  virtual void Reset();

  virtual void SendToOne(const std::string& str_addr,
                              const CByteArray& c_message) {};

  virtual void SendToMany(const CByteArray& c_message);

  virtual void SendToAll(const CByteArray& c_payload) {};

private:
  CWiFiEquippedEntity* m_pcWiFiEquippedEntity;
  
};

} // namespace argos
#endif // LEO_WIFI_DEFAULT_ACTUATOR_H
