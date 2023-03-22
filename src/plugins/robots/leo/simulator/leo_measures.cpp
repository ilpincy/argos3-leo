/**
 * @file <argos3/plugins/robots/leo/simulator/leo_measures.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "leo_measures.h"

/****************************************/
/****************************************/

const Real LEO_BASE_RADIUS    = 0.0704;
const Real LEO_BASE_ELEVATION = 0.0047;
const Real LEO_BASE_HEIGHT    = 0.053;
const Real LEO_BASE_TOP       = LEO_BASE_ELEVATION + LEO_BASE_HEIGHT;

const Real LEO_WHEEL_RADIUS        = 0.021;
const Real LEO_WHEEL_DISTANCE      = 0.1054;
const Real LEO_HALF_WHEEL_DISTANCE = LEO_WHEEL_DISTANCE * 0.5;

const Real LEO_IR_SENSORS_RING_ELEVATION = 0.0145;
const Real LEO_IR_SENSORS_RING_RADIUS    = LEO_BASE_RADIUS;
const Real LEO_IR_SENSORS_RING_RANGE     = 0.12;

const Real LEO_ULTRASOUND_SENSORS_RING_ELEVATION = 0.0145;
const Real LEO_ULTRASOUND_SENSORS_RING_RADIUS    = LEO_BASE_RADIUS;
const CRange<Real> LEO_ULTRASOUND_SENSORS_RING_RANGE(0.25, 2.0);

const CVector2 LEO_IR_SENSORS_GROUND_OFFSET[4] = {
   CVector2(0.06140,  0.01),
   CVector2(0.02060,  0.059),
   CVector2(0.02060, -0.059),
   CVector2(0.06140, -0.01)
};

const CVector3 LEO_LEDS_OFFSET[3] = {
   CVector3( 0.04,  0.025, LEO_BASE_TOP),
   CVector3(-0.05,  0.000, LEO_BASE_TOP),
   CVector3( 0.04, -0.025, LEO_BASE_TOP)
};

const Real LEO_LIDAR_CENTER_ELEVATION   = 0.084;
const Real LEO_LIDAR_ELEVATION          = LEO_BASE_TOP + LEO_LIDAR_CENTER_ELEVATION;
const Real LEO_LIDAR_SENSORS_FAN_RADIUS = LEO_BASE_RADIUS;
const CRadians LEO_LIDAR_ANGLE_SPAN(ToRadians(CDegrees(210.0)));
const CRange<Real> LEO_LIDAR_SENSORS_RING_RANGE(0.02, 4.0);

/****************************************/
/****************************************/
