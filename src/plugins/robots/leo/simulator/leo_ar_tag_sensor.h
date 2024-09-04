#ifndef LEO_AR_TAG_SENSOR_H
#define LEO_AR_TAG_SENSOR_H

#include <map>
#include <string>

namespace argos {
class CLeoArTagSensor;
class CEmbodiedEntity;
} // namespace argos

#include <argos3/core/simulator/sensor.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_ar_tag_sensor.h>

namespace argos {
class CLeoArTagSensor : public CCI_LeoArTagSensor,
                           public CSimulatedSensor {

public:
  CLeoArTagSensor();
  ~CLeoArTagSensor() {}

  virtual void Init(TConfigurationNode &t_tree) {}
  virtual void Reset() {}
  void SetRobot(CComposableEntity &c_entity) {}
  void Update() {}
};
} // namespace argos
#endif // LEO_AR_TAG_SENSOR_H
