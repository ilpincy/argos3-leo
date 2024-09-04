#include "leo_ar_tag_sensor.h"
#include <argos3/core/simulator/simulator.h>

/****************************************/
/****************************************/

namespace argos {

REGISTER_SENSOR(
    CLeoArTagSensor, "leo_ar_tag", "default",
    "Ashay Aswale [ashayaswale@gmail.com]", "1.0",
    "An empty AR Tag for leo sensor.",

    "This sensor is an empty implementation and does not do anything. In\n"
    "controllers, you must include the ci_leo_ar_tag_sensor.h header.\n\n"

    "REQUIRED XML CONFIGURATION\n\n"
    "  <controllers>\n"
    "    ...\n"
    "    <my_controller ...>\n"
    "      ...\n"
    "      <sensors>\n"
    "        ...\n"
    "        <leo_ar_tag implementation=\"default\" />\n"
    "        ...\n"
    "      </sensors>\n"
    "      ...\n"
    "    </my_controller>\n"
    "    ...\n"
    "  </controllers>\n\n"

    "OPTIONAL XML CONFIGURATION\n\n"

    "None.\n",

    "Usable");

} // namespace argos
/****************************************/
/****************************************/
