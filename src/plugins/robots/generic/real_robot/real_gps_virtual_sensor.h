/**
 * @file <argos3/plugins/robots/generic/real_robot/real_gps_virtual_sensor.h>
 *
 * @author Bernard Mayeur <bmayeur@ulb.ac.be>
 */

#ifndef REAL_GPS_VIRTUAL_SENSOR_H
#define REAL_GPS_VIRTUAL_SENSOR_H

namespace argos {
   class CRealGPSVirtualSensor;
}

#include <argos3/core/utility/logging/argos_log.h>

#include "real_virtual_sensor.h"
#include <argos3/plugins/robots/generic/control_interface/ci_gps_virtual_sensor.h>

namespace argos {
   class CRealGPSVirtualSensor: virtual public CRealVirtualSensor, virtual public CCI_GPSVirtualSensor
   {
   public:

      /**
       * Constructor
       */
	   CRealGPSVirtualSensor();

      /**
       * Destructor
       */
      ~CRealGPSVirtualSensor();

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

   private:

      CVirtualSensorDataPosition2D m_cLastReading;
   };

}
#endif
