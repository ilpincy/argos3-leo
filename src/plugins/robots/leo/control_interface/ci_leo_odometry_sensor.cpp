/**
 * @file <argos3/plugins/robots/leo/control_interface/ci_leo_odometry_sensor.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "ci_leo_odometry_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {
   
  /****************************************/
  /****************************************/
   
#ifdef ARGOS_WITH_LUA
  void CCI_LeoOdometrySensor::CreateLuaState(lua_State* pt_lua_state) {
    // CLuaUtility::OpenRobotStateTable(pt_lua_state, "wheels");
    // CLuaUtility::AddToTable(pt_lua_state, "distance_left",  m_sReading.CoveredDistanceLeftWheel );
    // CLuaUtility::AddToTable(pt_lua_state, "distance_right", m_sReading.CoveredDistanceRightWheel);
    // CLuaUtility::AddToTable(pt_lua_state, "velocity_left",  m_sReading.VelocityLeftWheel        );
    // CLuaUtility::AddToTable(pt_lua_state, "velocity_right", m_sReading.VelocityRightWheel       );
    // CLuaUtility::AddToTable(pt_lua_state, "axis_length",    m_sReading.WheelAxisLength          );
    // CLuaUtility::CloseRobotStateTable(pt_lua_state);
  }
#endif

  /****************************************/
  /****************************************/

#ifdef ARGOS_WITH_LUA
  void CCI_LeoOdometrySensor::ReadingsToLuaState(lua_State* pt_lua_state) {
    // lua_getfield  (pt_lua_state, -1, "wheels"                        );
    // lua_pushnumber(pt_lua_state, m_sReading.CoveredDistanceLeftWheel );
    // lua_setfield  (pt_lua_state, -2, "distance_left"                 );
    // lua_pushnumber(pt_lua_state, m_sReading.CoveredDistanceRightWheel);
    // lua_setfield  (pt_lua_state, -2, "distance_right"                );
    // lua_pushnumber(pt_lua_state, m_sReading.VelocityLeftWheel        );
    // lua_setfield  (pt_lua_state, -2, "velocity_left"                 );
    // lua_pushnumber(pt_lua_state, m_sReading.VelocityRightWheel       );
    // lua_setfield  (pt_lua_state, -2, "velocity_right"                );
    // lua_pop       (pt_lua_state, 1                                   );
  }
#endif


  /****************************************/
  /****************************************/

  const CCI_LeoOdometrySensor::SReading& CCI_LeoOdometrySensor::GetReading() const {
    return m_sReading;
  }

  /****************************************/
  /****************************************/

}
