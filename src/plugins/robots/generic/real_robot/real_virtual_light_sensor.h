/**
 * @file <argos3/plugins/robots/generic/real_robot/real_virtual_light_sensor.h>
 *
 * @author Mattia Salvaro
 */

#ifndef REAL_VIRTUAL_LIGHT_SENSOR_H
#define REAL_VIRTUAL_LIGHT_SENSOR_H

namespace argos {
   class CRealVirtualLightSensor;

}

#include <argos3/plugins/robots/generic/control_interface/ci_light_sensor.h>
#include "real_virtual_sensor.h"
#include <argos3/core/utility/logging/argos_log.h>

namespace argos {

   class CRealVirtualLightSensor : virtual public CCI_LightSensor,
                                           public CRealVirtualSensor
   {
	   class CLightVirtualSensorReading : virtual public CVirtualSensorNetworkData {
	   	public:
	   		CLightVirtualSensorReading(){
	   			m_vecReading.resize(unVectorSize);
	   		}
	   		/**
	   		 * Constructor of a packet from another one
	   		 */
	   		CLightVirtualSensorReading(const CLightVirtualSensorReading & t_packet):
	   			m_vecReading(t_packet.m_vecReading) {}

	   		CLightVirtualSensorReading(const std::vector<Real>& vec_reading):
	   			m_vecReading(vec_reading) {}

	   		/**
	   		 * Redefine the "=" operator
	   		 */
	   		virtual CLightVirtualSensorReading& operator=(const CLightVirtualSensorReading & t_packet) {
	   		   if (&t_packet != this) {
	   			   m_vecReading = t_packet.m_vecReading;
	   		   }
	   		   return *this;
	   		}

	   		/*
	   		 * Transform the content of the class to a vector of char/UInt8
	   		 */
	   		virtual CByteArray Serialize() const{
	   			CByteArray vecBuffer ;
	   			if (m_vecReading.size() != unVectorSize)
	   				THROW_ARGOSEXCEPTION("CLightVirtualSensorReading : Incorrect number of readings")
	   			for (UInt32 i=0; i<unVectorSize; ++i)
	   				vecBuffer << m_vecReading[i];
	   			return vecBuffer;
	   		}

	   		/*
	   		 * Change the values of the class to apply the content of the vector
	   		 */
	   		virtual void Deserialize(CByteArray& vec_buffer){
	   			m_vecReading.clear();
	   			Real fReading;
	   			while(vec_buffer.Size()){
	   				vec_buffer >> fReading;
	   				m_vecReading.push_back(fReading);
	   			}
	   			SetVectorSize(m_vecReading.size());
	   		}

	   		void SetVectorSize(UInt32 un_new_size){
	   			unVectorSize = un_new_size;
	   		}
	   		static UInt32 unVectorSize;

	   		const std::vector<Real>& GetReadings(){
	   			return m_vecReading;
	   		}

	   		std::vector<Real> m_vecReading;
	   	};
   public:

      /**
       * Constructor
       */
      CRealVirtualLightSensor();

      /**
       * Destructor
       */
      ~CRealVirtualLightSensor();

      /**
       * Initialize the sensor
       *
       * @param t_node configuration node of the ground
       * sensor found in the xml configuration file
       */
      virtual void Init(TConfigurationNode& t_node);

      /**
       * @brief UpdateValues The metod that must be called in the main loop of the real robot
       * in order to update the readings of all the Virutal Sensors
       */
      virtual void UpdateValues();

      CLightVirtualSensorReading m_cReadings;

   };

}
#endif
