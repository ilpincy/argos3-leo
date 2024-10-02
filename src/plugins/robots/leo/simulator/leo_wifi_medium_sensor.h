#ifndef LEO_WIFI_MEDIUM_SENSOR_H
#define LEO_WIFI_MEDIUM_SENSOR_H

#include <map>
#include <string>

namespace argos {
class CLeoWiFiMediumSensor;
class CWiFiEquippedEntity;
class CEmbodiedEntity;
class CWiFiMedium;
} // namespace argos

#include <argos3/core/simulator/sensor.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_wifi_sensor.h>
#include <argos3/plugins/simulator/entities/wifi_equipped_entity.h>

namespace argos {

class CLeoWiFiMediumSensor : public CCI_LeoWiFiSensor, public CSimulatedSensor {

public:

  CLeoWiFiMediumSensor();
  ~CLeoWiFiMediumSensor() {}

  virtual void SetRobot(CComposableEntity& c_entity);

  virtual void Init(TConfigurationNode& t_tree);

  virtual void Update();

  virtual void Reset();

  virtual void Destroy();

  virtual void Enable();

  virtual void Disable();

      /**
       * Fills the given message vector and flushes the internal message queue
       */
      virtual void GetMessages(std::vector<CCI_LeoWiFiSensor::SMessage>& vec_messages);

      /**
       * Discards all the messages in the queue.
       */
      virtual void FlushMessages() {};

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

  CWiFiEquippedEntity*  m_pcWiFiEquippedEntity;
  CControllableEntity* m_pcControllableEntity;
  CWiFiMedium*          m_pcWiFiMedium;
  Real                 m_fDistanceNoiseStdDev;
  Real                 m_fPacketDropProb;
  CRandom::CRNG*       m_pcRNG;
  CSpace&              m_cSpace;
  bool                 m_bShowRays;

//   std::vector<CCI_LeoWiFiSensor::SMessage> m_vec_messages;

};

} // namespace argos
#endif // LEO_WIFI_MEDIUM_SENSOR_H
