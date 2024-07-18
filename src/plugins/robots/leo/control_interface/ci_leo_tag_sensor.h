#ifndef CI_LEO_TAG_SENSOR_H
#define CI_LEO_TAG_SENSOR_H

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>

namespace argos {
   class CCI_LeoTagSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>

namespace argos {

   class CCI_LeoTagSensor : public CCI_Sensor {

   public:

      struct SReading {
         int TagId;
         int Confidence;
         CVector3 Position;
         CQuaternion Orientation;
         
         SReading() {}
         
         SReading(const int& c_tag_id,
                  const int& c_confidence,
                  const CVector3& c_position,
                  const CQuaternion& c_orientation) :
            TagId(c_tag_id),
            Confidence(c_confidence),
            Position(c_position),
            Orientation(c_orientation) {}
      };

      typedef std::vector<SReading> TReadings;

      /**
       * Constructor
       */
      CCI_LeoTagSensor() {}

      /**
       * Destructor
       */
      virtual ~CCI_LeoTagSensor() {}

      /**
       * @brief Returns the reading of the encoder sensor
       * Returns the reading of the encoder sensor
       */
      const TReadings& GetReading() const;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:

      TReadings m_tReadings;
   };

}

#endif // CI_LEO_TAG_SENSOR_H
