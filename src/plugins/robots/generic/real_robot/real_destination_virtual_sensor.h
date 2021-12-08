/**
 * @file <argos3/plugins/robots/generic/real_robot/real_destination_virtual_sensor.h>
 *
 * @author Bernard Mayeur <bmayeur@ulb.ac.be>
 */

#ifndef REAL_DESTINATION_VIRTUAL_SENSOR_H
#define REAL_DESTINATION_VIRTUAL_SENSOR_H

namespace argos {
   class CRealDestinationVirtualSensor;

}

#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/plugins/robots/generic/control_interface/ci_destination_virtual_sensor.h>
#include "real_virtual_sensor.h"

namespace argos {

   class CRealDestinationVirtualSensor: public CCI_DestinationVirtualSensor, public CRealVirtualSensor
   {

   public:

      /**
       * Constructor
       */
	   CRealDestinationVirtualSensor();

      /**
       * Destructor
       */
      ~CRealDestinationVirtualSensor();

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
