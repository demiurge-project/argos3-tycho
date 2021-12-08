/* Include the controller definition */
#include "real_ground_sensor_sampler.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/

CRealGroundSensorSampler::CRealGroundSensorSampler() :
    m_pcVirtualGroundSensor(NULL),
    m_sLeftWheelSpeed(0),
    m_sRightWheelSpeed(0),
    m_nControlStep(0),
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
    m_bTurnLeft(true),
    BLACK(0),
    WHITE(1),
    GRAY(0.5),
    BLACKTHRESHOLD(0.10),
    WHITETHRESHOLD(0.90)

{}

/****************************************/
/****************************************/

void CRealGroundSensorSampler::Init(TConfigurationNode& t_node) {
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
    */
   //m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   //m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );



   /*
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

    DEBUG_FUNCTION_ENTER;
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
    } catch (CARGoSException ex) {}
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

    try {
       m_pcVirtualGroundSensor = GetSensor<CCI_GroundSensor>("virtual_ground_sensor");
    } catch (CARGoSException ex) {}



    m_unTimeSession = m_pcRNG->Uniform(CRange<UInt32>(0, 40));



    DEBUG_FUNCTION_EXIT;
}



/****************************************/
/****************************************/

void CRealGroundSensorSampler::ControlStep() {

    DEBUG_FUNCTION_ENTER;


    if (m_pcRGBLEDsActuator != NULL) {
        //m_pcRGBLEDsActuator->SetColors(CColor::RED);
    }
    else {
        //THROW_ARGOSEXCEPTION("m_pcRGBLEDsActuator == NULL");
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

    if (!(m_nControlStep%10)) {
        m_pcWheelsActuator->SetLinearVelocity(0, 0);
    }

    /****/
    if (m_pcProximitySensor != NULL) {
       // prints the proximity sensors readings
       DEBUG("[PROXIMITY]\t");
       for(size_t i = 0; i < 8; ++i) {

           if(i==2) continue;
           if(i==3) continue;
           if(i==4) continue;
           if(i==5) continue;

          DEBUG("(%.2f - %.2f)\t", m_pcProximitySensor->GetReading(i).Value,
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





    /***/

    if (m_pcGroundSensor != NULL) {
        CCI_EPuckGroundSensor::SReadings vecReadings = m_pcGroundSensor->GetReadings();
        std::vector<Real>* vecPrefilteredReadings = new std::vector<Real>();
        Real fValue;
        DEBUG("prefiltering\n");
        // Prefiltering
        for (size_t i=0; i<3; i++) {

            fValue = vecReadings[i];
            DEBUG("Value: %f\n", fValue);
            if (fValue <= BLACKTHRESHOLD) {
                fValue = BLACK;
            }
            else if (fValue >= WHITETHRESHOLD){
                fValue = WHITE;
                //uint unRandomIndex = rand() % 200;
                //printf("Random index: %d\n", unRandomIndex);
                //fValue = gray_samples[unRandomIndex];
                //printf("Gray sample: %f\n",fValue);
            }
            else {
                fValue = GRAY;
            }
            DEBUG("Prefiltered Value: %f\n", fValue);
            vecPrefilteredReadings->push_back(fValue);
        }
        DEBUG("after Prefiltering\n");

        // Print ground sensor value
        DEBUG("[GROUND]\t%d\t%f %f %f\t%f %f %f\n",m_nControlStep, vecReadings[0], vecReadings[1], vecReadings[2],
              vecPrefilteredReadings->at(0), vecPrefilteredReadings->at(1), vecPrefilteredReadings->at(2));
/*
        Real fAverageReading = 0;
        for(std::vector<Real>::iterator itReadings = vecReadings.begin(); itReadings != vecReadings.end(); itReadings ++)
        {
            DEBUG("Average reading: %f\tCurrent reading %f\n", fAverageReading, (*itReadings))
            fAverageReading = fAverageReading + (*itReadings);
        }

        fAverageReading = fAverageReading / vecReadings.size();
*/
        Real fAverageReading = (vecReadings.Left + vecReadings.Center + vecReadings.Right)/3;

        DEBUG("Virtual Ground Sensor: average reading = %f\n", fAverageReading);

        if (fAverageReading < 0.1) {
            //m_pcRGBLEDsActuator->SetColors(CColor::YELLOW);
        }
        else if (fAverageReading < 0.9) {
            //m_pcRGBLEDsActuator->SetColors(CColor::RED);
        }
        else {
            //m_pcRGBLEDsActuator->SetColors(CColor::BLUE);
        }
    }

    else {
        //THROW_ARGOSEXCEPTION("m_pcRGBLEDsActuator != NULL && m_pcGroundSensor != NULL");
    }



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
REGISTER_CONTROLLER(CRealGroundSensorSampler, "ground_sensor_sampler");
