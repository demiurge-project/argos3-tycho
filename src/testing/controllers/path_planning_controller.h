/*
 * pathPlanning.h
 *
 *  Created on: 27 juin 2014
 *      Author: bernard
 */

#ifndef PATHPLANNING_H_
#define PATHPLANNING_H_

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/generic/control_interface/ci_gps_virtual_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_destination_virtual_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_wheels_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_base_leds_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_rgb_leds_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_battery_sensor.h>
#include <argos3/plugins/robots/generic/real_robot/virtual_sensor_client.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray2.h>

#include "void_controller.h"
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CPathPlanningController : public CVoidController {
public:

   /* Class constructor. */
   CPathPlanningController();

   /* Class destructor. */
   virtual ~CPathPlanningController() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

   virtual CCI_GPSVirtualSensor* GetGPSSensor(){
	   if (!m_pcGPSSensor)
		   THROW_ARGOSEXCEPTION("CANNOT MAKE PATHPLANNING WITHOUT GPS SENSOR !");
	   return m_pcGPSSensor;
   }
   virtual CCI_DestinationVirtualSensor* GetDestinationSensor(){
	   if (!m_pcDestinationSensor)
		   THROW_ARGOSEXCEPTION("CANNOT MAKE PATHPLANNING WITHOUT DESTINATION SENSOR !");
	   return m_pcDestinationSensor;
   }

private:

   void followVector(CVector2 sum, Real f_speed=MAX_VELOCITY);
   void followVectorStraight(CVector2 c_vector_to_follow, Real f_speed=MAX_VELOCITY);
   void SetLinearRotation(CRadians fPos,CRadians fDest);
   CVector2 ParseProximity();

   CCI_EPuckWheelsActuator* m_pcWheelsActuator;
   CCI_EPuckProximitySensor* m_pcProximitySensor;

   CCI_GPSVirtualSensor* m_pcGPSSensor;
   CCI_DestinationVirtualSensor* m_pcDestinationSensor;

   CCI_EPuckBaseLEDsActuator* m_pcLEDActuator;
   CCI_EPuckRGBLEDsActuator* m_pcRGBLEDActuator;
   CCI_EPuckBatterySensor* m_pcBatterySensor;

   static const int MAX_VELOCITY = 14;
   Real m_fStopThreshold;
   Real m_fProximityThreshold;
   Real m_fProximityRepulsionForce;
   Real m_fProximityRepulsionForceNoise;
   Real m_fAngleStopThreshold;
   UInt32 m_fBatteryTimeStepThreshold;
   UInt32 m_unBatteryLowCounter;
   bool m_bForceBatteryChange;
   CVirtualSensorDataPosition2D m_cRechargeDest;
   UInt32 m_unMaxTickForceRandomWalk;
   argos::CRandom::CRNG* m_pcRNG;


};

#endif /* PATHPLANNING_H_ */
