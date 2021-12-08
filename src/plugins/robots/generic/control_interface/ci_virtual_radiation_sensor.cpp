/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_virtual_radiation_sensor.cpp>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */

#include "ci_virtual_radiation_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/
/*
#ifdef ARGOS_WITH_LUA
   void CCI_VirtualRadiationSensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::AddToTable(pt_lua_state, "Radiation", m_cReading.intensity);
      CLuaUtility::EndTable(pt_lua_state);
   }
*/
   /****************************************/
   /****************************************/
/*
   void CCI_VirtualRadiationSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
		CLuaUtility::AddToTable(pt_lua_state, "Radiation", m_cReading.Intensity);
		CLuaUtility::EndTable(pt_lua_state);
		lua_pop(pt_lua_state, 1);
   }
#endif
*/
}
