/**
 * @file <argos3/plugins/robots/generic/real_robot/real_virtual_rgb_ground_sensor.h>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */

#ifndef REAL_VIRTUAL_RGB_GROUND_SENSOR_H
#define REAL_VIRTUAL_RGB_GROUND_SENSOR_H

namespace argos {
   class CRealVirtualRGBGroundSensor;
}

#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/plugins/robots/generic/control_interface/ci_virtual_rgb_ground_sensor.h>
#include "real_virtual_sensor.h"

namespace argos {

   class CRealVirtualRGBGroundSensor: public CCI_VirtualRGBGroundSensor, public CRealVirtualSensor
   {

   public:

      /**
       * Constructor
       */
	   CRealVirtualRGBGroundSensor();

      /**
       * Destructor
       */
      ~CRealVirtualRGBGroundSensor();

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

   };

}
#endif
