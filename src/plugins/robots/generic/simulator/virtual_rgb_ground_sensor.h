/**
 * @file <argos3/plugins/robots/generic/real_robot/virtual_rgb_ground_sensor.h>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */

#ifndef VIRTUAL_RGB_GROUND_SENSOR_H
#define VIRTUAL_RGB_GROUND_SENSOR_H

namespace argos {
   class CVirtualRGBGroundSensor;
}

//#include <algorithm>

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/physics_engines/iridia_tracking_system/virtual_sensor_server/virtual_sensor_data.h>
//#include <argos3/plugins/robots/generic/control_interface/virtual_sensor_register.h>
#include <argos3/plugins/robots/generic/simulator/generic_virtual_sensor.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/sensor.h>
#include <argos3/core/simulator/space/space.h>

//#include <argos3/plugins/robots/generic/control_interface/ci_virtual_rgb_ground_sensor.h>
#include "../control_interface/ci_virtual_rgb_ground_sensor.h"

namespace argos {

class CVirtualRGBGroundSensor : virtual public CSimulatedSensor,
								  virtual public CGenericVirtualSensor,
								  virtual public CCI_VirtualRGBGroundSensor
{
public:
	CVirtualRGBGroundSensor();

   virtual ~CVirtualRGBGroundSensor() {}

   virtual const std::string ClassName() const;

   virtual void SetRobot(CComposableEntity& c_entity);

   virtual void Init(TConfigurationNode& t_tree);

   /**
    * @brief Update Performs regular Update PLUS updates the Virtual Sensor Data Structure
    */
   virtual void Update();


protected:
   CEmbodiedEntity* m_pcEmbodiedEntity;

   /** Reference to the space */
   CSpace& m_cSpace;

   /** Reference to floor entity */
   CFloorEntity* m_pcFloorEntity;
};

}


#endif
