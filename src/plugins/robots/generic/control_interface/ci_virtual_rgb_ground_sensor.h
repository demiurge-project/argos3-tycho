/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_virtual_rgb_ground_sensor.h>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */
#ifndef CCI_VIRTUAL_RGB_GROUND_SENSOR_H
#define CCI_VIRTUAL_RGB_GROUND_SENSOR_H


namespace argos {
   class CCI_VirtualRGBGroundSensor;
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
   class CCI_VirtualRGBGroundSensor : virtual public CCI_Sensor
   {
	   class CVirtualRGBGroundSensorReading : virtual public CVirtualSensorNetworkData {
	   	public:
	   		CVirtualRGBGroundSensorReading():R(0),G(0),B(0){}
	   		/**
	   		 * Constructor of a packet from another one
	   		 */
	   		CVirtualRGBGroundSensorReading(const CVirtualRGBGroundSensorReading & t_packet):
	   			R(t_packet.R),G(t_packet.G),B(t_packet.B) {}

	   		CVirtualRGBGroundSensorReading(const UInt8 &R, const UInt8 &G, const UInt8 &B):
	   			R(R),G(G),B(B) {}

	   		/**
	   		 * Redefine the "=" operator
	   		 */
	   		virtual CVirtualRGBGroundSensorReading& operator=(const CVirtualRGBGroundSensorReading & t_packet) {
	   			//TODO
	   			if (&t_packet != this) {
	   				R = t_packet.R;
	   				G = t_packet.G;
	   				B = t_packet.B;
	   		   }
	   		   return *this;
	   		}
	   		/*
	   		 * Transform the content of the class to a vector of char/UInt8
	   		 */
	   		virtual CByteArray Serialize() const{
	   			CByteArray vecBuffer;
	   			vecBuffer << R << G << B;
	   			return vecBuffer;
	   		}

	   		/*
	   		 * Change the values of the class to apply the content of the vector
	   		 */
	   		virtual void Deserialize(CByteArray& vec_buffer){
	   			vec_buffer >> B >> G >> R;
	   		}

	   		UInt8 R,G,B;
	   	};
   public:
	   CCI_VirtualRGBGroundSensor() {}
      virtual ~CCI_VirtualRGBGroundSensor() {}

      inline const CVirtualRGBGroundSensorReading& GetReading() const {
         return m_cReading;
      }

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state){};
      virtual void ReadingsToLuaState(lua_State* pt_lua_state){};
#endif
   protected:
      CVirtualRGBGroundSensorReading m_cReading;
   };

}

#endif
