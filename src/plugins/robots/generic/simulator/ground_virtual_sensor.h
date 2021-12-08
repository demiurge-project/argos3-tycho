/**
 * @file <argos3/plugins/robot/generic/simulator/ground_virtual_sensor.h>
 *
 * Provides the Virtual Ground Sensor
 *
 * @author Mattia Salvaro
 */

#ifndef GROUND_VIRTUAL_SENSOR_H
#define GROUND_VIRTUAL_SENSOR_H

#include <algorithm>

#include <argos3/core/simulator/entity/composable_entity.h>

#include <argos3/plugins/simulator/entities/ground_sensor_equipped_entity.h>
//#include <argos3/plugins/simulator/sensors/ground_rotzonly_sensor.h>
#include <argos3/plugins/robots/generic/simulator/ground_rotzonly_sensor.h>

#include <argos3/plugins/simulator/physics_engines/iridia_tracking_system/virtual_sensor_server/virtual_sensor_data.h>
#include <argos3/plugins/robots/generic/simulator/generic_virtual_sensor.h>

namespace argos {

class CGroundVirtualSensor : public CGenericVirtualSensor,
                             public CGroundRotZOnlySensor
{

public :
	class CGroundVirtualSensorReading : virtual public CVirtualSensorNetworkData {
	public:
		CGroundVirtualSensorReading(){
		}
		/**
		 * Constructor of a packet from another one
		 */
		CGroundVirtualSensorReading(const CGroundVirtualSensorReading & t_packet):
			m_vecReading(t_packet.m_vecReading) {}

		CGroundVirtualSensorReading(const std::vector<Real>& vec_reading):
			m_vecReading(vec_reading) {}

		/**
		 * Redefine the "=" operator
		 */
		virtual CGroundVirtualSensorReading& operator=(const CGroundVirtualSensorReading & t_packet) {
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
			if (m_vecReading.size() != unVectorSize){
				LOGERR << "CGroundVirtualSensorReading : Incorrect number of readings\nExpecting " << unVectorSize << ", got " << m_vecReading.size() << std::endl;
				THROW_ARGOSEXCEPTION("CGroundVirtualSensorReading : Incorrect number of readings")
			}
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
	protected:
		std::vector<Real> m_vecReading;
	};

public:

   CGroundVirtualSensor();

   virtual ~CGroundVirtualSensor() {}

   virtual void SetRobot(CComposableEntity& c_entity);

   virtual void Init(TConfigurationNode& t_tree);

   /**
    * @brief Update Performs regular Update PLUS updates the Virutal Sensor Data Structure
    */
   virtual void Update();

private:

   /**
    * @brief m_vecLastReading Record of the previous reading
    */
   CGroundVirtualSensorReading m_cReading;

};

}


#endif
