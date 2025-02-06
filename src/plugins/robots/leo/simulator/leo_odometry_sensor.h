#ifndef LEO_ODOMETRY_SENSOR_H
#define LEO_ODOMETRY_SENSOR_H

#include <map>
#include <string>

namespace argos {
class CLeoOdometrySensor;
class CEmbodiedEntity;
} // namespace argos

#include <argos3/core/simulator/sensor.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_odometry_sensor.h>

namespace argos {
class CLeoOdometrySensor : public CCI_LeoOdometrySensor,
                           public CSimulatedSensor {

public:
  CLeoOdometrySensor();
  ~CLeoOdometrySensor() {}

  virtual void Init(TConfigurationNode &t_tree);
  virtual void Reset();
  void SetRobot(CComposableEntity &c_entity);
  void Update();

protected:
  /** Reference to embodied entity associated to this sensor */
  CEmbodiedEntity *m_pcEmbodiedEntity;

  /** Random number generator */
  CRandom::CRNG *m_pcRNG;

  /** Whether to add noise or not */
  bool m_bAddNoise;

  /** Noise range on position */
  CRange<Real> m_cPosNoiseRange;

  /** Noise range on angle */
  CRange<CRadians> m_cAngleNoiseRange;

  /** Noise range on axis */
  CRange<Real> m_cAxisNoiseRange;

  CRadians m_cAngle;
  CVector3 m_cAxis;
};
} // namespace argos
#endif // LEO_ODOMETRY_SENSOR_H
