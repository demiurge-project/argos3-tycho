/*
 * void_controller.h
 *
 *  Created on: 27 juin 2014
 *      Author: bernard
 */

#ifndef VOID_H_
#define VOID_H_

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/generic/control_interface/ci_gps_virtual_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_destination_virtual_sensor.h>
#include <argos3/plugins/robots/generic/real_robot/virtual_sensor_client.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray2.h>
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CVoidController : public CCI_Controller {
public:

   /* Class constructor. */
	CVoidController();

   /* Class destructor. */
   virtual ~CVoidController() {}

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
   CCI_GPSVirtualSensor* m_pcGPSSensor;
   CCI_DestinationVirtualSensor* m_pcDestinationSensor;
};

#endif /* VOID_H_ */
