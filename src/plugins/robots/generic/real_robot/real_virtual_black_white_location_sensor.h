/**
 * @file <argos3/plugins/robots/generic/real_robot/real_virtual_black_white_location_sensor.h>
 *
 * @author Andreagiovanni Reina <areina@ulb.ac.be>
 */

#ifndef REAL_VIRTUAL_BLACK_WHITE_LOCATION_SENSOR_H
#define REAL_VIRTUAL_BLACK_WHITE_LOCATION_SENSOR_H

namespace argos {
   class CRealVirtualBlackWhiteLocationSensor;

}

#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/plugins/robots/generic/control_interface/ci_virtual_black_white_location_sensor.h>
#include "real_virtual_sensor.h"

namespace argos {

   class CRealVirtualBlackWhiteLocationSensor: public CCI_VirtualBlackWhiteLocationSensor, public CRealVirtualSensor
   {

   public:

      /**
       * Constructor
       */
	   CRealVirtualBlackWhiteLocationSensor();

      /**
       * Destructor
       */
      ~CRealVirtualBlackWhiteLocationSensor();

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
