/*
 * Author: Miquel Kegeleirs
 */

#ifndef PATHPLANNING_RVR
#define PATHPLANNING_RVR

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray2.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_wheels_actuator.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_velocity_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_quaternion_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_rgb_leds_actuator.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_proximity_sensor.h>

#include <string>
#include <sstream>
#include <math.h>
#include <vector>

// ROS libraries
#include "ros/ros.h"
// wheels
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
// LEDs
#include <rvr_ros/Leds.h>
// terabee
#include "teraranger_array/RangeArray.h"

#include "void_controller.h"
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CPathPlanningControllerRVR : public CVoidController {
public:

   /* Class constructor. */
   CPathPlanningControllerRVR();

   /* Class destructor. */
   virtual ~CPathPlanningControllerRVR() {}

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

   /* Handler for proximity sensor data */
   virtual void TerarangerHandler(const teraranger_array::RangeArray &msg);

private:
   // ROS methods
   virtual void InitRos(); // must be called in Init
   void followVector(CVector2 sum, Real f_speed=MAX_VELOCITY);
   void followVectorStraight(CVector2 c_vector_to_follow, Real f_speed=MAX_VELOCITY);
   void SetLinearRotation(CRadians fPos,CRadians fDest);
   CVector2 ParseProximity();
   void SetColors(const CColor& color);
   void SetVelocity(const Real& f_leftSpeed, const Real& f_rightspeed);


   CCI_RVRWheelsActuator* m_pcWheelsActuator;
   CCI_RVRVelocitySensor* m_pcVelocitySensor;
   CCI_RVRRGBLEDsActuator* m_pcLEDsActuator;
   CCI_RVRProximitySensor* m_pcProximitySensor;

   CCI_GPSVirtualSensor* m_pcGPSSensor;
   CCI_DestinationVirtualSensor* m_pcDestinationSensor;

   static const int MAX_VELOCITY = 30;
   Real m_fStopThreshold;
   Real m_fProximityThreshold;
   Real m_fProximityRepulsionForce;
   Real m_fProximityRepulsionForceNoise;
   Real m_fAngleStopThreshold;
   CVirtualSensorDataPosition2D m_cRechargeDest;
   UInt32 m_unMaxTickForceRandomWalk;
   argos::CRandom::CRNG* m_pcRNG;

   // Proximity sensors subscriber
    ros::Subscriber prox_sub;
   // Actuators publishers
   ros::Publisher vel_pub;
   std_msgs::Float32MultiArray vel_msg;

   ros::Publisher led_pub;
   rvr_ros::Leds led_msg;

   /* Wheel speed. */
   Real leftWheelVelocity;
   Real rightWheelVelocity;

   /* This is the default wheel velocity.
   It is usually parsed from the XML file. */
   Real m_fDefaultWheelVelocity;

   /*
   * The proximity sensors input.
   */
   CCI_RVRProximitySensor::TReadings m_sProximityInput;

};

#endif
