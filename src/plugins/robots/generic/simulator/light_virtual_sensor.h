/**
 * @file <argos3/plugins/robot/generic/simulator/light_virtual_sensor.h>
 *
 * Provides the Virtual Light Sensor
 *
 * @author Mattia Salvaro
 */

#ifndef LIGHT_VIRTUAL_SENSOR_H
#define LIGHT_VIRTUAL_SENSOR_H

#include <algorithm>

#include <argos3/core/simulator/entity/composable_entity.h>

#include <argos3/plugins/robots/generic/control_interface/ci_light_sensor.h>
//#include <argos3/plugins/simulator/sensors/light_default_sensor.h>
#include <argos3/plugins/robots/generic/simulator/light_default_sensor.h>
#include <argos3/plugins/simulator/entities/light_sensor_equipped_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/physics_engines/iridia_tracking_system/virtual_sensor_server/virtual_sensor_data.h>
#include <argos3/plugins/robots/generic/simulator/generic_virtual_sensor.h>

namespace argos {

class CLightVirtualSensor : public CGenericVirtualSensor,
                            public CLightDefaultSensor
{
	class CLightVirtualSensorReading : virtual public CVirtualSensorNetworkData {
	public:
		CLightVirtualSensorReading(){
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
			m_vecReading.resize(unVectorSize);
		}
		static UInt32 unVectorSize;

		std::vector<Real> m_vecReading;
	};
public:

   CLightVirtualSensor();

   virtual ~CLightVirtualSensor() {}

   virtual void SetRobot(CComposableEntity& c_entity);

   virtual void Init(TConfigurationNode& t_tree);

   /**
    * @brief Update Performs regular Update PLUS updates the Virutal Sensor Data Structure
    */
   virtual void Update();

protected:

    CEmbodiedEntity* m_pcEmbodiedEntity;

private:

   /**
    * @brief m_vecLastReading Record of the previous reading
    */
   CLightVirtualSensorReading m_cReading;
};

}


#endif
