/**
 * @file <argos3/plugins/robots/generic/real_robot/virtual_radiation_sensor.h>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */

#ifndef VIRTUAL_RADIATION_SENSOR_H
#define VIRTUAL_RADIATION_SENSOR_H

namespace argos {
   class CVirtualRadiationSensor;
}

//#include <algorithm>

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/physics_engines/iridia_tracking_system/virtual_sensor_server/virtual_sensor_data.h>
#include <argos3/plugins/robots/generic/simulator/generic_virtual_sensor.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/sensor.h>
#include <argos3/core/simulator/space/space.h>

//#include <argos3/plugins/robots/generic/control_interface/ci_virtual_radiation_sensor.h>
#include "../control_interface/ci_virtual_radiation_sensor.h"

namespace argos {

class CVirtualRadiationSensor : virtual public CSimulatedSensor,
								  virtual public CGenericVirtualSensor,
								  virtual public CCI_VirtualRadiationSensor
{
public:
	CVirtualRadiationSensor();

   virtual ~CVirtualRadiationSensor() {}

   virtual const std::string ClassName() const;

   virtual void SetRobot(CComposableEntity& c_entity);

   virtual void Init(TConfigurationNode& t_tree);

   /**
    * @brief Update Performs regular Update PLUS updates the Virtual Sensor Data Structure
    */
   virtual void Update();


protected:

   void IdentifyRadioactiveMaterial();

   CEmbodiedEntity* m_pcEmbodiedEntity;

   CEmbodiedEntity* m_pcRadioactiveMaterialEntity;

   /** Reference to the space */
   CSpace& m_cSpace;

};

}


#endif
