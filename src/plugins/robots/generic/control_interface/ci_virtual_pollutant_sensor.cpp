/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_virtual_pollutant_sensor.cpp>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */

#include "ci_virtual_pollutant_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/
/*
#ifdef ARGOS_WITH_LUA
   void CCI_VirtualPollutantSensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::AddToTable(pt_lua_state, "Pollutant", m_cReading.Value);
      CLuaUtility::EndTable(pt_lua_state);
   }
*/
   /****************************************/
   /****************************************/
/*
   void CCI_VirtualPollutantSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
		CLuaUtility::AddToTable(pt_lua_state, "Pollutant", m_cReading.Value);
		CLuaUtility::EndTable(pt_lua_state);
		lua_pop(pt_lua_state, 1);
   }
#endif
*/
}
