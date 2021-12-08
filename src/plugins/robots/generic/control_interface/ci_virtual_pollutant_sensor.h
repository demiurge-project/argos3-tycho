/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_virtual_pollutant_sensor.h>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */
#ifndef CCI_VIRTUAL_POLLUTANT_SENSOR_H
#define CCI_VIRTUAL_POLLUTANT_SENSOR_H


namespace argos {
   class CCI_VirtualPollutantSensor;
}

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/virtual_sensor_data_position2d.h>//network_data.h>
#include <argos3/core/utility/datatypes/byte_array.h>
#include <argos3/core/utility/logging/argos_log.h>
#ifdef ARGOS_WITH_LUA
    #include <argos3/core/wrappers/lua/lua_utility.h>
#endif


namespace argos {
   class CCI_VirtualPollutantSensor : virtual public CCI_Sensor
   {
	   class CVirtualPollutantSensorReading : virtual public CVirtualSensorNetworkData {
	   	public:
		   CVirtualPollutantSensorReading():Value(0){}
	   		/**
	   		 * Constructor of a packet from another one
	   		 */
		   CVirtualPollutantSensorReading(const CVirtualPollutantSensorReading & t_packet):
	   			Value(t_packet.Value) {}

		   CVirtualPollutantSensorReading(const Real &fValue):
	   			Value(fValue) {}

	   		/**
	   		 * Redefine the "=" operator
	   		 */
	   		virtual CVirtualPollutantSensorReading& operator=(const CVirtualPollutantSensorReading & t_packet) {
	   			if (&t_packet != this) {
	   				Value = t_packet.Value;
	   		   }
	   		   return *this;
	   		}
	   		/*
	   		 * Transform the content of the class to a vector of char/UInt8
	   		 */
	   		virtual CByteArray Serialize() const{
	   			CByteArray vecBuffer;
	   			vecBuffer << Value;
	   			return vecBuffer;
	   		}

	   		/*
	   		 * Change the values of the class to apply the content of the vector
	   		 */
	   		virtual void Deserialize(CByteArray& vec_buffer){
	   			vec_buffer >> Value;
	   		}

	   		Real Value;
	   	};

  public:
	   struct PollutantParams {
		   CDegrees cDirectionAngle;
		   CDegrees cWideAngle;
		   Real fMaxDistance;
		   UInt32 uRotationTimestep;
		   CDegrees cRotationAngle;
		   PollutantParams() : fMaxDistance(0), uRotationTimestep(0) {}
	   };

	   CCI_VirtualPollutantSensor() {}
	   virtual ~CCI_VirtualPollutantSensor() {}

	   inline const CVirtualPollutantSensorReading& GetReading() const {
		   return m_cReading;
	   }

	   inline const PollutantParams& GetParams() const {
		   return m_cParam;
	   }

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state){};
      virtual void ReadingsToLuaState(lua_State* pt_lua_state){};
#endif
   protected:
      CVirtualPollutantSensorReading m_cReading;

      PollutantParams m_cParam;

   };

}

#endif
