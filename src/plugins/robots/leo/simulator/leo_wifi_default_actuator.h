#ifndef LEO_WIFI_DEFAULT_ACTUATOR_H
#define LEO_WIFI_DEFAULT_ACTUATOR_H

#include <map>
#include <string>

namespace argos {
class CLeoWifiActuator;
} // namespace argos

#include <argos3/core/simulator/actuator.h>
// #include <argos3/core/simulator/space/space.h>
// #include <argos3/core/utility/math/range.h>
// #include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_wifi_actuator.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>

namespace argos {
class CLeoWifiActuator : public CCI_LeoWiFiActuator, public CSimulatedActuator {

public:
  CLeoWifiActuator() {};
  virtual ~CLeoWifiActuator() {}

  virtual void SetRobot(CComposableEntity &c_entity);
  virtual void Update();
  virtual void Reset();

private:
  CRABEquippedEntity* m_pcRangeAndBearingEquippedEntity;
  
};

} // namespace argos
#endif // LEO_WIFI_DEFAULT_ACTUATOR_H
