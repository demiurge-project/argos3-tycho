/*
 * void.cpp
 *
 *  Created on: 27 juin 2014
 *      Author: bernard
 */

/* Include the controller definition */
#include "void_controller.h"

/****************************************/
/****************************************/

CVoidController::CVoidController() :
    m_pcGPSSensor(NULL),
    m_pcDestinationSensor(NULL)
{}

/****************************************/
/****************************************/

void CVoidController::Init(TConfigurationNode& t_node) {
    try {
    	m_pcGPSSensor = GetSensor<CCI_GPSVirtualSensor>("gps_virtual_sensor");
    } catch (CARGoSException &ex) {}
    try {
	    m_pcDestinationSensor = GetSensor<CCI_DestinationVirtualSensor>("destination_virtual_sensor");
	} catch (CARGoSException &ex) {}
}

/****************************************/
/****************************************/

void CVoidController::ControlStep() {
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
#ifdef ARGOS_DYNAMIC_LIBRARY_LOADING
   REGISTER_CONTROLLER(CVoidController, "void_controller");
#endif
