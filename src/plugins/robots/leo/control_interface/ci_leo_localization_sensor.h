#ifndef CI_LEO_LOCALIZATION_SENSOR_H
#define CI_LEO_LOCALIZATION_SENSOR_H

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>

namespace argos {
   class CCI_LeoLocalizationSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>

namespace argos {

   class CCI_LeoLocalizationSensor : public CCI_Sensor {

   public:

      struct SReading {
         CVector3 Position;
         CQuaternion Orientation;
         
         SReading() {}
         
         SReading(const CVector3& c_position,
                  const CQuaternion& c_orientation) :
            Position(c_position),
            Orientation(c_orientation) {}
      };

      /**
       * Constructor
       */
      CCI_LeoLocalizationSensor() {}

      /**
       * Destructor
       */
      virtual ~CCI_LeoLocalizationSensor() {}

      /**
       * @brief Returns the reading of the encoder sensor
       * Returns the reading of the encoder sensor
       */
      const SReading& GetReading() const;

   protected:

      SReading m_sReading;
   };

}

#endif // CI_LEO_LOCALIZATION_SENSOR_H
