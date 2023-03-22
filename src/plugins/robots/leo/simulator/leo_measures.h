/**
 * @file <argos3/plugins/robots/leo/simulator/leo_measures.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef LEO_MEASURES_H
#define LEO_MEASURES_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>

using namespace argos;

extern const Real LEO_BASE_RADIUS;
extern const Real LEO_BASE_ELEVATION;
extern const Real LEO_BASE_HEIGHT;
extern const Real LEO_BASE_TOP;

extern const Real LEO_WHEEL_RADIUS;
extern const Real LEO_WHEEL_DISTANCE;
extern const Real LEO_HALF_WHEEL_DISTANCE;

extern const Real LEO_IR_SENSORS_RING_ELEVATION;
extern const Real LEO_IR_SENSORS_RING_RADIUS;
extern const Real LEO_IR_SENSORS_RING_RANGE;

extern const Real LEO_ULTRASOUND_SENSORS_RING_ELEVATION;
extern const Real LEO_ULTRASOUND_SENSORS_RING_RADIUS;
extern const CRange<Real> LEO_ULTRASOUND_SENSORS_RING_RANGE;

extern const CVector2 LEO_IR_SENSORS_GROUND_OFFSET[4];

extern const CVector3 LEO_LEDS_OFFSET[3];

extern const Real LEO_LIDAR_ELEVATION;
extern const Real LEO_LIDAR_SENSORS_FAN_RADIUS;
extern const CRadians LEO_LIDAR_ANGLE_SPAN;
extern const CRange<Real> LEO_LIDAR_SENSORS_RING_RANGE;

#endif
