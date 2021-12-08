/* Include the controller definition */
#include "its_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/

CITSController::CITSController() :
    m_sLeftWheelSpeed(0),
    m_sRightWheelSpeed(0),
    m_nControlStep(0),
    m_pcVirtualGroundSensor(NULL),
    m_pcVirtualLightSensor(NULL),
    m_pcWheelsActuator(NULL),
    m_pcLEDsActuator(NULL),
    m_pcRGBLEDsActuator(NULL),
    m_pcRABActuator(NULL),
    m_pcIRComActuator(NULL),
    m_pcProximitySensor(NULL),
    m_pcLightSensor(NULL),
    m_pcGroundSensor(NULL),
    m_pcRABSensor(NULL),
    m_pcIRComSensor(NULL),
    m_pcRNG(argos::CRandom::CreateRNG("argos")),
    m_unTimeSession(0),
    m_unTimeCounter(0),
    m_bTurning(false),
    m_bTurnLeft(false)

{}

/****************************************/
/****************************************/

void CITSController::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.

   //m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   //m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );

    //m_pcVirtualGroundSensor = GetSensor<CCI_VirtualGroundSensor>("virtual_ground_sensor");


    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   //GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   //m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   //GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   //GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

    //DEBUG_FUNCTION_ENTER;

    m_strFilename = this->GetId() + std::string(".txt");
    m_cOutStream.open(m_strFilename.c_str());
    if (!m_cOutStream.is_open()) {
        THROW_ARGOSEXCEPTION("Could not open output file\n")
    }


    /* print the id of the robot */
    DEBUG("[ROBOT]\t%s\n", GetId().c_str());
    /* actuators */
    try {
       m_pcWheelsActuator = GetActuator<CCI_EPuckWheelsActuator>("epuck_wheels");
    } catch (CARGoSException ex) {}
    try {
       m_pcLEDsActuator = GetActuator<CCI_EPuckBaseLEDsActuator>("epuck_base_leds");
    } catch (CARGoSException ex) {}
    try {
       m_pcRGBLEDsActuator = GetActuator<CCI_EPuckRGBLEDsActuator>("epuck_rgb_leds");
    } catch (CARGoSException ex) {
        DEBUG("%s\n", ex.what());
    }
    try {
       m_pcRABActuator = GetActuator<CCI_EPuckRangeAndBearingActuator>("epuck_range_and_bearing");
    } catch (CARGoSException ex) {}
    try {
       m_pcIRComActuator = GetActuator<CCI_EPuckIRComActuator>("epuck_ircom");
    } catch (CARGoSException ex) {}
    /* sensors */
    try {
       m_pcProximitySensor = GetSensor<CCI_EPuckProximitySensor>("epuck_proximity");
    } catch (CARGoSException ex) {}
    try {
       m_pcLightSensor = GetSensor<CCI_EPuckLightSensor>("epuck_light");
    } catch (CARGoSException ex) {}
    try {
       m_pcGroundSensor = GetSensor<CCI_EPuckGroundSensor>("epuck_ground");
    } catch (CARGoSException ex) {}
    try {
       m_pcRABSensor = GetSensor<CCI_EPuckRangeAndBearingSensor>("epuck_range_and_bearing");
    } catch (CARGoSException ex) {}
    try {
       m_pcIRComSensor = GetSensor<CCI_EPuckIRComSensor>("epuck_ircom");
    } catch (CARGoSException ex) {}

    /**/
    // New for Virtual Sensors
    try {
       m_pcVirtualGroundSensor = GetSensor<CCI_GroundSensor>("virtual_ground_sensor");
    } catch (CARGoSException ex) {
        DEBUG("%s\n", ex.what());
    }
    /***/
    try {
       m_pcVirtualLightSensor = GetSensor<CCI_LightSensor>("virtual_light_sensor");
    } catch (CARGoSException ex) {
        DEBUG("%s\n", ex.what());
    }
    /**/
    m_unTimeSession = m_pcRNG->Uniform(CRange<UInt32>(0, 40));


    //DEBUG_FUNCTION_EXIT;
}



/****************************************/
/****************************************/

void CITSController::ControlStep() {

    DEBUG_FUNCTION_ENTER;
   /* Get readings from proximity sensor *
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   * Sum them together *
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();
    * If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    *
   CRadians cAngle = cAccumulator.Angle();
   if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_fDelta ) {
      * Go straight *
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   }
   else {
      * Turn, depending on the sign of the angle *
      if(cAngle.GetValue() > 0.0f) {
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
      }
      else {
         m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
      }
   }
   */

    if (m_pcRGBLEDsActuator != NULL) {
        //DEBUG("m_pcRGBLEDsActuator != NULL");
        m_pcRGBLEDsActuator->SetColors(CColor::RED);

    }
    else {
        //THROW_ARGOSEXCEPTION("m_pcRGBLEDsActuator == NULL");
    }

    if (m_pcVirtualGroundSensor != NULL) {
        //DEBUG("m_pcVirtualGroundSensor != NULL")
    }
    else {
        //THROW_ARGOSEXCEPTION("m_pcVirtualGroundSensor == NULL");
    }
    if (m_bTurning && m_bTurnLeft) {
        m_pcWheelsActuator->SetLinearVelocity(-10, 10);
    }
    else if (m_bTurning && !m_bTurnLeft) {
        m_pcWheelsActuator->SetLinearVelocity(10, -10);
    }
    else {
        m_pcWheelsActuator->SetLinearVelocity(10, 10);
    }

    DEBUG("TIME: %d\n", m_unTimeCounter);

    /****/
    if (m_pcProximitySensor != NULL) {
       // prints the proximity sensors readings
       DEBUG("[PROXIMITY]\n");
       for(size_t i = 0; i < 8; ++i) {

           if(i==2) continue;
           if(i==3) continue;
           if(i==4) continue;
           if(i==5) continue;

          DEBUG("(%.2f - %.2f)\n", m_pcProximitySensor->GetReading(i).Value,
                 ToDegrees(m_pcProximitySensor->GetReading(i).Angle).GetValue());
          if (m_pcProximitySensor->GetReading(i).Value > 0.2 && !m_bTurning) {
              m_bTurning = true;
              m_unTimeCounter = 0;
              if (i==0) {
                  m_bTurnLeft = false;
                  m_unTimeSession = 5;

              }
              else if (i==1) {
                  m_bTurnLeft = false;
                 m_unTimeSession = 2;
              }
              else if (i==6){
                  m_bTurnLeft = true;
                 m_unTimeSession  = 2;
              }
              else if (i==7){
                  m_bTurnLeft = true;
                 m_unTimeSession  = 5;
              }
              //m_unTimeSession = m_pcRNG->Uniform(CRange<UInt32>(1, 5));
          }
       }
       DEBUG("\n");
    }

    DEBUG("TIME: %d\n", m_unTimeCounter);
    /***/

    if(m_pcVirtualLightSensor != NULL) {
        std::vector<Real> vecReadings = m_pcVirtualLightSensor->GetReadings();
        DEBUG("[LIGHT]\n");
        for (UInt8 unCounter=0; unCounter < vecReadings.size(); unCounter++) {
            DEBUG("%f\n", vecReadings[unCounter]);
        }
    }
    else {
        THROW_ARGOSEXCEPTION("m_pcVirtualLightSensor == NULL");
    }

    /***/

    if (m_pcRGBLEDsActuator != NULL && m_pcVirtualGroundSensor != NULL) {
        std::vector<Real> vecReadings = m_pcVirtualGroundSensor->GetReadings();
        DEBUG("vecReadings.size() = %d\n", vecReadings.size());
        if (vecReadings.size() == 0) {
            DEBUG_FUNCTION_EXIT;
            //THROW_ARGOSEXCEPTION("vecReadings.size()=0\n");
            return;
        }

        DEBUG("[GROUND]\t%d\t", m_nControlStep);
        m_cOutStream << m_nControlStep;
        for (size_t i=0; i<vecReadings.size(); i++) {
            DEBUG("%f ", vecReadings[i]);
            m_cOutStream << " " << vecReadings[i];
        }
        DEBUG("\n");
        m_cOutStream << "\n";

        Real fAverageReading = 0;
        for(std::vector<Real>::iterator itReadings = vecReadings.begin(); itReadings != vecReadings.end(); itReadings ++)
        {
            //DEBUG("Average reading: %f\tCurrent reading %f\n", fAverageReading, (*itReadings))
            fAverageReading = fAverageReading + (*itReadings);
        }

        fAverageReading = fAverageReading / vecReadings.size();

        DEBUG("Virtual Ground Sensor: average reading = %f\n", fAverageReading);

        if (fAverageReading < 0.1) {
            m_pcRGBLEDsActuator->SetColors(CColor::YELLOW);
        }
        else if (fAverageReading < 0.9) {
            m_pcRGBLEDsActuator->SetColors(CColor::RED);
        }
        else {
            m_pcRGBLEDsActuator->SetColors(CColor::BLUE);
        }
    }

    else {
        //THROW_ARGOSEXCEPTION("m_pcRGBLEDsActuator != NULL && m_pcVirtualGroundSensor != NULL");
    }

    DEBUG("TIME: %d\n", m_unTimeCounter);


    if(m_unTimeCounter == m_unTimeSession) {
        m_bTurning = !m_bTurning;
        if (m_bTurning) {
            m_unTimeSession = m_pcRNG->Uniform(CRange<UInt32>(1, 20));
        }
        else {
            m_unTimeSession = m_pcRNG->Uniform(CRange<UInt32>(1, 40));
        }

        m_unTimeCounter = 0;
    }



    m_unTimeCounter++;
    m_nControlStep++;
    /*
     * GetReadings() read values from datastructure
     *
     * light a led
     */

    DEBUG_FUNCTION_EXIT;
}

/****************************************/
/****************************************/

void CITSController::Reset()
{
    DEBUG_FUNCTION_ENTER;

    m_sLeftWheelSpeed = 0;
    m_sRightWheelSpeed = 0;
    m_nControlStep = 0;
    m_unTimeSession = 0;
    m_unTimeCounter = 0;
    m_bTurning = false;
    m_bTurnLeft = false;

    if(m_pcVirtualGroundSensor != NULL) {
        m_pcVirtualGroundSensor->Reset();
    }
    if(m_pcWheelsActuator != NULL) {
        m_pcWheelsActuator->Reset();
        m_pcWheelsActuator->SetLinearVelocity(0,0);
    }
    if(m_pcLEDsActuator != NULL) {
        m_pcLEDsActuator->Reset();
    }
    if(m_pcRGBLEDsActuator != NULL) {
        //m_pcRGBLEDsActuator->Reset();
        m_pcRGBLEDsActuator->SetColors(CColor::BLUE);
    }
    if(m_pcRABActuator != NULL) {
        m_pcRABActuator->Reset();
    }
    if(m_pcIRComActuator != NULL) {
        m_pcIRComActuator->Reset();
    }
    if(m_pcProximitySensor != NULL) {
        m_pcProximitySensor->Reset();
    }
    if(m_pcLightSensor != NULL) {
        m_pcLightSensor->Reset();
    }
    if(m_pcGroundSensor != NULL) {
        m_pcGroundSensor->Reset();
    }
    if(m_pcRABSensor != NULL) {
        m_pcRABSensor->Reset();
    }
    if(m_pcIRComSensor != NULL) {
        m_pcIRComSensor->Reset();
    }

    m_cOutStream.close();
    m_cOutStream.open(m_strFilename.c_str());
    DEBUG_FUNCTION_EXIT;
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
REGISTER_CONTROLLER(CITSController, "its_controller");
