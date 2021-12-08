/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_gps_virtual_sensor.cpp>
 *
 * @author Bernard Mayeur <bmayeur@gmail.com>
 */

#include "ci_gps_virtual_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/

/*#ifdef ARGOS_WITH_LUA
   void CCI_GPSVirtualSensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::StartTable(pt_lua_state, "GPS");
      CLuaUtility::AddToTable(pt_lua_state, "XRange", m_cReading.XRange);
      CLuaUtility::AddToTable(pt_lua_state, "YRange", m_cReading.YRange);
      CLuaUtility::AddToTable(pt_lua_state, "Bearing", m_cReading.Bearing);
      CLuaUtility::EndTable(pt_lua_state);
   }
#endif*/

   /****************************************/
   /****************************************/

/*#ifdef ARGOS_WITH_LUA
   void CCI_GPSVirtualSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
		lua_getfield(pt_lua_state, -1, "GPS");
		CLuaUtility::StartTable(pt_lua_state, "GPS");
		CLuaUtility::AddToTable(pt_lua_state, "XRange", m_cReading.XRange);
		CLuaUtility::AddToTable(pt_lua_state, "YRange", m_cReading.YRange);
		CLuaUtility::AddToTable(pt_lua_state, "Bearing", m_cReading.Bearing);
		CLuaUtility::EndTable(pt_lua_state);
		//lua_pop(pt_lua_state, 1);
   }
#endif*/
}
