/**
 * @file <argos3/plugins/robots/e-puck/real_robot/real_virtual_sensor.h>
 *
 * @author Mattia Salvaro
 */

#ifndef REAL_VIRTUAL_SENSOR_H
#define REAL_VIRTUAL_SENSOR_H

#include <string>

#include <argos3/plugins/robots/generic/real_robot/virtual_sensor_client.h>
//#include "virtual_sensor_client.h"
#include <argos3/plugins/robots/generic/control_interface/virtual_sensor_register.h>
//#include "../control_interface/virtual_sensor_register.h"

namespace argos {

   class CRealVirtualSensor
   {

   public:

      /**
       * Constructor
       */
       CRealVirtualSensor() ;

      /**
       * Destructor
       */
      virtual ~CRealVirtualSensor(){}

       /**
        * @brief UpdateValues The metod that must be called in the main loop of the real robot
        * in order to update the readings of all the Virutal Sensors
        */
       virtual void UpdateValues() = 0;

   protected:
       UInt8 m_unSensorId;
       CVirtualSensorClient & m_cVirtualSensorClient;
   };

}






#endif
