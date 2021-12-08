/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_virtual_radiation_sensor.h>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */
#ifndef CCI_VIRTUAL_RADIATION_SENSOR_H
#define CCI_VIRTUAL_RADIATION_SENSOR_H


namespace argos {
   class CCI_VirtualRadiationSensor;
}

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/virtual_sensor_data_position2d.h>//network_data.h>
#include <argos3/core/utility/datatypes/byte_array.h>
#include <argos3/core/utility/logging/argos_log.h>
#ifdef ARGOS_WITH_LUA
    #include <argos3/core/wrappers/lua/lua_utility.h>
#endif


namespace argos {
   class CCI_VirtualRadiationSensor : virtual public CCI_Sensor
   {
	   class CVirtualRadiationSensorReading : virtual public CVirtualSensorNetworkData {
	   	public:
		   CVirtualRadiationSensorReading():Intensity(0){}
	   		/**
	   		 * Constructor of a packet from another one
	   		 */
		   CVirtualRadiationSensorReading(const CVirtualRadiationSensorReading & t_packet):
	   			Intensity(t_packet.Intensity) {}

		   CVirtualRadiationSensorReading(const Real &fIntensity):
	   			Intensity(fIntensity) {}

	   		/**
	   		 * Redefine the "=" operator
	   		 */
	   		virtual CVirtualRadiationSensorReading& operator=(const CVirtualRadiationSensorReading & t_packet) {
	   			if (&t_packet != this) {
	   				Intensity = t_packet.Intensity;
	   		   }
	   		   return *this;
	   		}
	   		/*
	   		 * Transform the content of the class to a vector of char/UInt8
	   		 */
	   		virtual CByteArray Serialize() const{
	   			CByteArray vecBuffer;
	   			vecBuffer << Intensity;
	   			return vecBuffer;
	   		}

	   		/*
	   		 * Change the values of the class to apply the content of the vector
	   		 */
	   		virtual void Deserialize(CByteArray& vec_buffer){
	   			vec_buffer >> Intensity;
	   		}

	   		Real Intensity;
	   	};
   public:
	   CCI_VirtualRadiationSensor() {}
      virtual ~CCI_VirtualRadiationSensor() {}

      inline const CVirtualRadiationSensorReading& GetReading() const {
         return m_cReading;
      }

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state){};
      virtual void ReadingsToLuaState(lua_State* pt_lua_state){};
#endif
   protected:
      CVirtualRadiationSensorReading m_cReading;
   };

}

#endif
