/**
 * @file <argos3/plugins/robots/generic/control_interface/ci_virtual_black_white_location_sensor.h>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */
#ifndef CCI_VIRTUAL_BLACK_WHITE_LOCATION_SENSOR_H
#define CCI_VIRTUAL_BLACK_WHITE_LOCATION_SENSOR_H


namespace argos {
   class CCI_VirtualBlackWhiteLocationSensor;
}

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/virtual_sensor_data_position2d.h>//network_data.h>
#include <argos3/core/utility/datatypes/byte_array.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/vector2.h>

#ifdef ARGOS_WITH_LUA
    #include <argos3/core/wrappers/lua/lua_utility.h>
#endif


namespace argos {
   class CCI_VirtualBlackWhiteLocationSensor : virtual public CCI_Sensor
   {
	   class CVirtualBlackWhiteLocationSensorReading : virtual public CVirtualSensorNetworkData {
	   	public:
		   CVirtualBlackWhiteLocationSensorReading(){}
	   		/**
	   		 * Constructor of a packet from another one
	   		 */
		   CVirtualBlackWhiteLocationSensorReading(const CVirtualBlackWhiteLocationSensorReading & t_packet): vWhiteLocation(t_packet.vWhiteLocation),
          vBlackLocation(t_packet.vBlackLocation) { }

		   CVirtualBlackWhiteLocationSensorReading(const CVector2 &vWhite, const CVector2 &vBlack): vWhiteLocation(vWhite), vBlackLocation(vBlack) {}

	   		/**
	   		 * Redefine the "=" operator
	   		 */
	   		virtual CVirtualBlackWhiteLocationSensorReading& operator=(const CVirtualBlackWhiteLocationSensorReading & t_packet) {
	   			if (&t_packet != this) {
                    vBlackLocation = t_packet.vBlackLocation;
                    vWhiteLocation = t_packet.vWhiteLocation;
	   		   }
	   		   return *this;
	   		}
	   		/*
	   		 * Transform the content of the class to a vector of char/UInt8
	   		 */
	   		virtual CByteArray Serialize() const{
	   			CByteArray vecBuffer;
                vecBuffer << vWhiteLocation.GetX() << vWhiteLocation.GetY() << vBlackLocation.GetX() << vBlackLocation.GetY();
                return vecBuffer;
	   		}

	   		/*
	   		 * Change the values of the class to apply the content of the vector
	   		 */
	   		virtual void Deserialize(CByteArray& vec_buffer){
                Real x,y;
                // Deserialize white vector
                try {
                    vec_buffer >> x >> y;
                    vWhiteLocation.Set(x, y);
                    //Deserialiaz black vector
                    vec_buffer >> x >> y;
                    vBlackLocation.Set(x, y);
                } catch (CARGoSException e) {
                    LOGERR << "[WARNING] VirtualBlackWhiteLocationSensor: no data in buffer received from the tracking system. Should we keep in memory the previous data?" << std::endl;
                }

	   		}

            CVector2 vWhiteLocation;
            CVector2 vBlackLocation;
	   	};
   public:
	   CCI_VirtualBlackWhiteLocationSensor() {}
      virtual ~CCI_VirtualBlackWhiteLocationSensor() {}

      inline const CVirtualBlackWhiteLocationSensorReading& GetReading() const {
         return m_cReading;
      }

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state){};
      virtual void ReadingsToLuaState(lua_State* pt_lua_state){};
#endif
   protected:
      CVirtualBlackWhiteLocationSensorReading m_cReading;
   };

}

#endif
