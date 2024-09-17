#ifndef LEO_WIFI_MEDIUM_SENSOR_H
#define LEO_WIFI_MEDIUM_SENSOR_H

#include <map>
#include <string>

namespace argos {
class CLeoWiFiSensor;
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
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

namespace argos {
class CLeoWiFiSensor : public CCI_LeoWiFiSensor, public CSimulatedSensor {

public:
  CLeoWiFiSensor();
  ~CLeoWiFiSensor() {}

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
      virtual void GetMessages(std::vector<SMessage>& vec_messages) {};

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
  
  // public:

  //     struct SPacket {
  //        Real Range;
  //        CRadians HorizontalBearing;
  //        /**
  //         * The vertical bearing is defined as the angle between the local
  //         * robot XY plane and the message source position, i.e., the elevation
  //         * in math jargon. This is different from the inclination, which is the
  //         * angle between the azimuth vector (robot local Z axis) and
  //         * the vector to the message source. Elevation = 90 degrees - Inclination.
  //         */
  //        CRadians VerticalBearing;
  //        CByteArray Data;

  //        SPacket();
  //     };

  //     typedef std::vector<SPacket> TReadings;

  //  protected:

  //     TReadings m_tReadings;

  //  };

private:

  CRABEquippedEntity*  m_pcRangeAndBearingEquippedEntity;
  CControllableEntity* m_pcControllableEntity;
  CRABMedium*          m_pcRangeAndBearingMedium;
  Real                 m_fDistanceNoiseStdDev;
  Real                 m_fPacketDropProb;
  CRandom::CRNG*       m_pcRNG;
  CSpace&              m_cSpace;
  bool                 m_bShowRays;
  CCI_RangeAndBearingSensor::TReadings m_tReadings;

};

} // namespace argos
#endif // LEO_WIFI_MEDIUM_SENSOR_H
