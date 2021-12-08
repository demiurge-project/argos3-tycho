/**
 * @file <argos3/plugins/robots/generic/real_robot/real_virtual_pollutant_sensor.h>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */

#ifndef REAL_VIRTUAL_POLLUTANT_SENSOR_H
#define REAL_VIRTUAL_POLLUTANT_SENSOR_H

namespace argos {
   class CRealVirtualPollutantSensor;

}

#include <argos3/core/utility/logging/argos_log.h>

//#include <argos3/plugins/robots/generic/control_interface/ci_virtual_pollutant_sensor.h>
#include "../control_interface/ci_virtual_pollutant_sensor.h"
#include "real_virtual_sensor.h"

namespace argos {

   class CRealVirtualPollutantSensor: public CCI_VirtualPollutantSensor, public CRealVirtualSensor
   {

   public:

      /**
       * Constructor
       */
	   CRealVirtualPollutantSensor();

      /**
       * Destructor
       */
      ~CRealVirtualPollutantSensor();

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
