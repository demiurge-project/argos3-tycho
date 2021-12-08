/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_gps_virtual_sensor.h>
 *
 * @author Bernard Mayeur <bmayeur@gmail.com>
 */
#ifndef CCI_GPS_VIRTUAL_SENSOR_H
#define CCI_GPS_VIRTUAL_SENSOR_H

namespace argos {
   class CCI_GPSVirtualSensor;
   class CSimulatedSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/datatypes/byte_array.h>
#include <argos3/plugins/robots/generic/control_interface/virtual_sensor_data_position2d.h>

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   class CCI_GPSVirtualSensor : virtual public CCI_Sensor
   {
   public:

      virtual ~CCI_GPSVirtualSensor() {}

      inline const CVirtualSensorDataPosition2D& GetReading() const {
         return m_cReading;
      }

      inline UInt32 GetTickTimeTillLastUpdate() {
    	  return m_unTimeTillUpdate;
      }
#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state){};
      virtual void ReadingsToLuaState(lua_State* pt_lua_state){};
#endif

   protected:
      CVirtualSensorDataPosition2D m_cReading;
      UInt32 m_unTimeTillUpdate;
   };

}

#endif
