#include "ci_leo_posetwist_actuator.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   /*
    * The stack must have one value:
    * 1. x coordinate of the velocity (a number)
    */
   int LuaSetLeoLinearVelocity(lua_State* pt_lua_state) {
      /* Check parameters */
      if(lua_gettop(pt_lua_state) != 1) {
         return luaL_error(pt_lua_state, "robot.posetwist.set_linear_velocity() expects 1 argument");
      }
      luaL_checktype(pt_lua_state, 1, LUA_TNUMBER);
      /* Perform action */
      CLuaUtility::GetDeviceInstance<CCI_LeoPoseTwistActuator>(pt_lua_state, "posetwist")->
         SetLinearVelocity(lua_tonumber(pt_lua_state, 1));
      return 0;
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   /*
    * The stack must have one value:
    * 1. x coordinate of the velocity (a number)
    */
   int LuaSetLeoAngularVelocity(lua_State* pt_lua_state) {
      /* Check parameters */
      if(lua_gettop(pt_lua_state) != 1) {
         return luaL_error(pt_lua_state, "robot.posetwist.set_angular_velocity() expects 1 argument");
      }
      luaL_checktype(pt_lua_state, 1, LUA_TNUMBER);
      /* Perform action */
      CLuaUtility::GetDeviceInstance<CCI_LeoPoseTwistActuator>(pt_lua_state, "posetwist")->
         SetAngularVelocity(CRadians(lua_tonumber(pt_lua_state, 1)));
      return 0;
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_LeoPoseTwistActuator::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "posetwist");
      CLuaUtility::AddToTable(pt_lua_state, "_instance", this);
      CLuaUtility::AddToTable(pt_lua_state, "set_linear_velocity", &LuaSetLeoLinearVelocity);
      CLuaUtility::AddToTable(pt_lua_state, "set_angular_velocity", &LuaSetLeoAngularVelocity);
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif

   /****************************************/
   /****************************************/

}
