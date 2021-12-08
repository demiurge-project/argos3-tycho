/**
 * @file <argos3/plugins/robot/e-puck/simulator/epuck_ground_virtual_sensor.cpp>
 *
 * Provides the E-Puck Virutal Ground Sensor
 *
 * @author Mattia Salvaro
 */


#include "epuck_ground_virtual_sensor.h"



namespace argos {

/****************************************/
/****************************************/

CEPuckGroundVirtualSensor::CEPuckGroundVirtualSensor()
    : SENSOR_ID(CVirtualSensorRegister::GetVirtualSensorId("EPuckGroundVirtualSensor")),
      m_bNoiseModelOn(false)
{}

/****************************************/
/****************************************

CGroundVirtualSensor::~CGroundVirtualSensor()
{

}

/****************************************/
/****************************************/

void CEPuckGroundVirtualSensor::Init(TConfigurationNode &t_tree)
{
    CGroundRotZOnlySensor::Init(t_tree);

    GetNodeAttributeOrDefault(t_tree, "sensor_model_file", m_strRealSensorModelFile, std::string(""));

    // If a model file for the sensor is specified, load the parameters and build the model
    if (!m_strRealSensorModelFile.empty()) {
        m_bNoiseModelOn = true;
        BuildNoiseModel(m_strRealSensorModelFile);
        InitClassTable();
    }

    SetRealExperiment(t_tree);

    // If the Virtual Sensor is not included in the Virtual Sensor Table yet...
    if (!m_cVirtualSensorData.IsSensorAlreadyInTable(SENSOR_ID)) {
        // ... calculate the data size of this particular Virutal Sensor
        // Virtual Ground Sensor data is composed by a number of sensors,
        // each of them specified by a value of type Real
        m_unVirtualSensorDataSize = m_pcGroundSensorEntity->GetNumSensors() * sizeof(Real);
        //DEBUG("VSD size: %d\n", m_unVirtualSensorDataSize);
        // Add the Virtual Sensor Entry in the Virtual Sensor Table
        m_cVirtualSensorData.AddVirtualSensorEntry(SENSOR_ID, m_unVirtualSensorDataSize);
    }

    Update();

    /* prints the ground sensors readings */
/*    DEBUG("[GROUND]\t%.2f\t%.2f\t%.2f\n",
          m_vecLastReading[0],
          m_vecLastReading[1],
          m_vecLastReading[2]);
    DEBUG("\n\n");*/


}

/****************************************/
/****************************************/


void CEPuckGroundVirtualSensor::Update()
{
    CEPuckGroundRotZOnlySensor::Update();

    // If the updating condition is satisfied
    if (UpdatingCondition()) {

        // Send readings to the robot
        char byteDataBuffer[m_unVirtualSensorDataSize + 1];
        UInt32 unIndexBuffer = 0;
        byteDataBuffer[unIndexBuffer] = SENSOR_ID;
        unIndexBuffer = unIndexBuffer + 1;

        // If the noise model is on, substitute the simulator readings with the model generated readings
        if (m_bNoiseModelOn && !m_vecLastReading.empty()) {
            // Generate the input
            alglib::real_1d_array cInput;
            alglib::real_1d_array cResult;
            SReadings pfInput[6];
            pfInput[0] = m_sReadings[0];
            pfInput[1] = m_sReadings[1];
            pfInput[2] = m_sReadings[2];
            pfInput[3] = m_vecLastReading[0];
            pfInput[4] = m_vecLastReading[1];
            pfInput[5] = m_vecLastReading[2];

            cInput.setcontent(6, pfInput);

            // Feed the model with input
            alglib::mnlprocess(m_cLogitNoiseModel, cInput, cResult);

            for (UInt8 unIndex=0; unIndex<cResult.length(); unIndex++) {
                //DEBUG("Class: %d\tProb: %f\n", unIndex, cResult[unIndex]);
            }
            UInt8 unChosenClass = SelectProbabilisticClass(cResult);
            LOG << "Chosen class: %d\n", unChosenClass;
            ApplyClassValuesToReadings(unChosenClass);
        }

        if(m_bRealExperiment) {
            std::vector<Real>::const_iterator itRealVector;
            // For each reading add it in byte form to the byte data buffer
            for (itRealVector = m_sReadings.begin(); itRealVector != m_sReadings.end(); itRealVector++)
            {
                //DEBUG("[GROUND] %f\n", (*itRealVector));
                memcpy(byteDataBuffer + unIndexBuffer,&(*itRealVector),sizeof(Real));
                unIndexBuffer = unIndexBuffer + sizeof(Real);
            }
            // Append the Virtual Sensor byte data buffer in the Virtual Sensor Data Struct
            m_cVirtualSensorData.AppendVirtualSensorData(byteDataBuffer, unIndexBuffer, m_pairITSAndRobotId.second);
        }
    }

    // Give to the last readings the value of the current readings
    // in order to be compared with future readings in the next step.
    m_vecLastReading = m_sReadings;
}

/****************************************/
/****************************************/

REGISTER_SENSOR(CEPuckGroundVirtualSensor,
                "epuck_virtual_ground_sensor", "default",
                "Mattia Salvaro",
                "1.0",
                "The general robot virtual ground sensor.",
                "This sensor accesses the general virtual ground sensor.\n",
                "Usable"
       );

}
