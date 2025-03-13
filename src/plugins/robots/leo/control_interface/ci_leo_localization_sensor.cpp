#include "ci_leo_localization_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {
  const CCI_LeoLocalizationSensor::SReading& CCI_LeoLocalizationSensor::GetReading() const {
    return m_sReading;
  }

}
