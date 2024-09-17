#ifndef LEO_WIFI_MEDIUM_SENSOR_H
#define LEO_WIFI_MEDIUM_SENSOR_H

#include <map>
#include <string>

namespace argos {
class CLeoWifiSensor;
class CRABEquippedEntity;
class CEmbodiedEntity;
class CRABMedium;
} // namespace argos

#include <argos3/core/simulator/sensor.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_wifi_sensor.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>

namespace argos {
class CLeoWifiSensor : public CCI_LeoWiFiSensor, public CSimulatedSensor {

public:
  CLeoWifiSensor();
  ~CLeoWifiSensor() {}

  virtual void SetRobot(CComposableEntity& c_entity);

  virtual void Init(TConfigurationNode& t_tree);

  virtual void Update();

  virtual void Reset();

  virtual void Destroy();

  virtual void Enable();

  virtual void Disable();

  /**
   * Returns true if the rays must be shown in the GUI.
   * @return true if the rays must be shown in the GUI.
   */
  inline bool IsShowRays() {
      return m_bShowRays;
  }

  /**
   * Sets whether or not the rays must be shown in the GUI.
   * @param b_show_rays true if the rays must be shown, false otherwise
   */
  inline void SetShowRays(bool b_show_rays) {
      m_bShowRays = b_show_rays;
  }

private:

  CRABEquippedEntity*  m_pcRangeAndBearingEquippedEntity;
  CControllableEntity* m_pcControllableEntity;
  CRABMedium*          m_pcRangeAndBearingMedium;
  Real                 m_fDistanceNoiseStdDev;
  Real                 m_fPacketDropProb;
  CRandom::CRNG*       m_pcRNG;
  CSpace&              m_cSpace;
  bool                 m_bShowRays;

};

} // namespace argos
#endif // LEO_WIFI_MEDIUM_SENSOR_H
