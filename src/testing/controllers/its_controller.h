/*
 * AUTHOR: Mattia Salvaro
 *
 * This generic controler is meant to be used with the
 * Iridia Tracking System physics engine.
 *
 * The controller does absolutely nothing.
 * The Tracking System is the real driver of the robots.
 *
 * This controller is meant to be used with the XML files:
 *    testing/real_time_experiment.argos
 */

#ifndef ITS_CONTROLLER_H
#define ITS_CONTROLLER_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/rng.h>

#include <argos3/plugins/robots/generic/control_interface/ci_ground_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_light_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_wheels_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_base_leds_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_rgb_leds_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_ircom_actuator.h>

#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_light_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_ground_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_ircom_sensor.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CITSController : public CCI_Controller {

public:

   /* Class constructor. */
   CITSController();

   /* Class destructor. */
   virtual ~CITSController() {}

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
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

private:

   SInt32 m_sLeftWheelSpeed;
   SInt32 m_sRightWheelSpeed;

   UInt32 m_nControlStep;

   CCI_GroundSensor* m_pcVirtualGroundSensor;
   CCI_LightSensor* m_pcVirtualLightSensor;

   CCI_EPuckWheelsActuator* m_pcWheelsActuator;
   CCI_EPuckBaseLEDsActuator* m_pcLEDsActuator;
   CCI_EPuckRGBLEDsActuator* m_pcRGBLEDsActuator;
   CCI_EPuckRangeAndBearingActuator* m_pcRABActuator;
   CCI_EPuckIRComActuator* m_pcIRComActuator;

   CCI_EPuckProximitySensor* m_pcProximitySensor;
   CCI_EPuckLightSensor* m_pcLightSensor;
   CCI_EPuckGroundSensor* m_pcGroundSensor;
   CCI_EPuckRangeAndBearingSensor* m_pcRABSensor;
   CCI_EPuckIRComSensor* m_pcIRComSensor;

   argos::CRandom::CRNG* m_pcRNG;
   UInt32 m_unTimeSession;
   UInt32 m_unTimeCounter;
   bool m_bTurning;
   bool m_bTurnLeft;

   std::string m_strFilename;
   std::ofstream m_cOutStream;

};

#endif
