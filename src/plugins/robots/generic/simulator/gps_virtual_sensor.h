/**
 * @file <argos3/plugins/robots/generic/real_robot/gps_virtual_sensor.h>
 *
 * @author Bernard Mayeur [bmayeur@ulb.ac.be]
 */

#ifndef GPS_VIRTUAL_SENSOR_H
#define GPS_VIRTUAL_SENSOR_H

namespace argos {
   class CGPSVirtualSensor;
}

#include <algorithm>

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/physics_engines/iridia_tracking_system/virtual_sensor_server/virtual_sensor_data.h>
//#include <argos3/plugins/robots/generic/control_interface/virtual_sensor_register.h>
#include <argos3/plugins/robots/generic/control_interface/ci_gps_virtual_sensor.h>
#include <argos3/plugins/robots/generic/simulator/generic_virtual_sensor.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/sensor.h>

namespace argos {

class CGPSVirtualSensor : virtual public CSimulatedSensor,
					      virtual public CGenericVirtualSensor,
						  virtual public CCI_GPSVirtualSensor
{

public:

   CGPSVirtualSensor();

   virtual ~CGPSVirtualSensor() {}

   virtual const std::string ClassName() const;

   virtual void SetRobot(CComposableEntity& c_entity);

   virtual void Init(TConfigurationNode& t_tree);

   /**
    * @brief Update Performs regular Update PLUS updates the Virtual Sensor Data Structure
    */
   virtual void Update();

protected:

   CEmbodiedEntity* m_pcEmbodiedEntity;
};

}


#endif
