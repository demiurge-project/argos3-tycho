/**
 * @file <argos3/plugins/robot/e-puck/simulator/epuck_ground_virtual_sensor.h>
 *
 * Provides the E-Puck Virtual Ground Sensor
 *
 * @author Mattia Salvaro
 */

#ifndef EPUCK_GROUND_VIRTUAL_SENSOR_H
#define EPUCK_GROUND_VIRTUAL_SENSOR_H

#include <algorithm>

#include <argos3/core/simulator/entity/composable_entity.h>

#include <argos3/plugins/robots/e-puck/simulator/epuck_ground_rotzonly_sensor.h>
#include <argos3/plugins/robots/generic/simulator/ground_virtual_sensor.h>

namespace argos {

class CEPuckGroundVirtualSensor : public CEPuckGroundRotZOnlySensor,
                                  public CGroundVirtualSensor
{

public:

   CEPuckGroundVirtualSensor();

   virtual ~CEPuckGroundVirtualSensor() {}

   /**
    * @brief Update Performs regular Update PLUS updates the Virutal Sensor Data Structure
    */
   virtual void Update();


private:

   /**
    * @brief m_vecLastReading Record of the previous reading
    */
   CCI_EPuckGroundSensor::SReadings m_vecLastReading;
};

}


#endif
