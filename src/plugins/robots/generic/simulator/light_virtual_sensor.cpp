/**
 * @file <argos3/plugins/robot/generic/simulator/light_virtual_sensor.cpp>
 *
 * Provides the Virutal Light Sensor
 *
 * @author Mattia Salvaro
 */


#include "light_virtual_sensor.h"



namespace argos {

/****************************************/
/****************************************/

static CRange<Real> UNIT(0.0f, 1.0f);
UInt32 CLightVirtualSensor::CLightVirtualSensorReading::unVectorSize=0;

/****************************************/
/****************************************/

CLightVirtualSensor::CLightVirtualSensor()
    : CGenericVirtualSensor(), CLightDefaultSensor()
{
	m_unSensorId = CVirtualSensorRegister::GetVirtualSensorId("LightVirtualSensor");
}

/****************************************/
/****************************************/

void CLightVirtualSensor::Init(TConfigurationNode &t_tree)
{
    CLightDefaultSensor::Init(t_tree);
    // Calculate the data size of this particular Virtual Sensor
    m_cReading.SetVectorSize(m_pcLightEntity->GetNumSensors());
	m_unVirtualSensorDataSize = m_cReading.Serialize().Size();

    // If the Virtual Sensor is not included in the Virtual Sensor Table yet...
    if (!m_cVirtualSensorData.IsSensorAlreadyInTable(m_unSensorId)) {
        // Add the Virtual Sensor Entry in the Virtual Sensor Table
        m_cVirtualSensorData.AddVirtualSensorEntry(m_unSensorId, m_unVirtualSensorDataSize);
    }

    Update();
}

/****************************************/
/****************************************/

void CLightVirtualSensor::SetRobot(CComposableEntity &c_entity)
{
    // Bind the instance of the Virtual Sensor with the robot
    CLightDefaultSensor::SetRobot(c_entity);
    // Set the pair of ITS and Robot ID starting from the Argos ID
    m_strRobotArgosId = c_entity.GetId();
    try{
        m_pcEmbodiedEntity = &(dynamic_cast<CFootBotEntity&>(c_entity).GetEmbodiedEntity());
    }
    catch (const std::bad_cast& e){
        try{
            m_pcEmbodiedEntity = &(dynamic_cast<CEPuckEntity&>(c_entity).GetEmbodiedEntity());
        }
        catch (const std::bad_cast& e){
            throw CARGoSException("Unknown robot");
        }
    }

    GetITSAndRobotIdFromArgosId(m_pairITSAndRobotId, m_strRobotArgosId);
}

/****************************************/
/****************************************/

void CLightVirtualSensor::Update()
{
    /* Erase readings */
    for(size_t i = 0; i < m_tReadings.size(); ++i)  m_tReadings[i] = 0.0f;
    /* Ray used for scanning the environment for obstacles */
    CRay3 cScanningRay;
    CVector3 cRayStart;
    CVector3 cSensorToLight;
    /* Buffers to contain data about the intersection */
    SEmbodiedEntityIntersectionItem sIntersection;
    /* List of light entities */
    //CSpace::TMapPerType& mapLights = m_cSpace.GetEntitiesByType("light");
    CSpace::TMapPerTypePerId::iterator itLights = m_cSpace.GetEntityMapPerTypePerId().find("light");
    if (itLights != m_cSpace.GetEntityMapPerTypePerId().end()) {
        CSpace::TMapPerType& mapLights = itLights->second;

        /* Go through the sensors */
        for(UInt32 i = 0; i < m_tReadings.size(); ++i) {
            /* Set ray start */
            cRayStart = m_pcLightEntity->GetSensor(i).Position;
            cRayStart.Rotate(m_pcEmbodiedEntity->GetOriginAnchor().Orientation);
            cRayStart += m_pcEmbodiedEntity->GetOriginAnchor().Position;
            /* Go through all the light entities */
            for(CSpace::TMapPerType::iterator it = mapLights.begin();
                it != mapLights.end();
                ++it) {
                /* Get a reference to the light */
                CLightEntity& cLight = *any_cast<CLightEntity*>(it->second);
                /* Consider the light only if it has non zero intensity */
                if(cLight.GetIntensity() > 0.0f) {
                    /* Set ray end to light position */
                    cScanningRay.Set(cRayStart, cLight.GetPosition());
                    /* Check occlusions */
                    if(! GetClosestEmbodiedEntityIntersectedByRay(sIntersection,
                                                                  cScanningRay)) {
                        /* No occlusion, the light is visibile */
                        if(m_bShowRays) {
                            m_pcControllableEntity->AddCheckedRay(false, cScanningRay);
                        }
                        /* Calculate reading */
                        cScanningRay.ToVector(cSensorToLight);
                        m_tReadings[i] += CalculateReading(cSensorToLight.Length(),
                                                           cLight.GetIntensity());
                    }
                    else {
                        /* There is an occlusion, the light is not visible */
                        if(m_bShowRays) {
                            m_pcControllableEntity->AddIntersectionPoint(cScanningRay,
                                                                         sIntersection.TOnRay);
                            m_pcControllableEntity->AddCheckedRay(true, cScanningRay);
                        }
                        // Check if the occlusion is caused by the the robot itself
                        //DEBUG("Intersection id: %s\nRobot id: %s\n", sIntersection.IntersectedEntity->GetId().c_str(), m_pcEmbodiedEntity->GetId().c_str());
                        if (sIntersection.IntersectedEntity->GetId().compare(m_pcEmbodiedEntity->GetId())==0) {
                            // Check if the direction of the scanning ray is within [-pi/2, pi/2]
                            //cScanningRay.ToVector(cSensorToLight);
                            cScanningRay.GetDirection(cSensorToLight);
                            CVector3 cSensor3DVector = m_pcLightEntity->GetSensor(i).Direction;
                            /*
                    DEBUG("Sensor %d direction: %f %f %f\n", i,
                          cSensor3DVector.GetXAngle().SignedNormalize().GetValue()*CRadians::RADIANS_TO_DEGREES,
                          cSensor3DVector.GetYAngle().SignedNormalize().GetValue()*CRadians::RADIANS_TO_DEGREES,
                          cSensor3DVector.GetZAngle().SignedNormalize().GetValue()*CRadians::RADIANS_TO_DEGREES);
                          */
                            CQuaternion cRobotOrientation = m_pcEmbodiedEntity->GetOriginAnchor().Orientation;
                            CRadians cRobotRadians;
                            CRadians cNotCare;
                            CRadians cNotCare2;
                            cRobotOrientation.ToEulerAngles(cRobotRadians, cNotCare2, cNotCare);
                            //DEBUG("Robot orientation: %f\n", cRobotRadians.GetValue()*CRadians::RADIANS_TO_DEGREES);
                            cSensor3DVector.Rotate(m_pcEmbodiedEntity->GetOriginAnchor().Orientation);
                            /*
                    DEBUG("Sensor %d direction: %f %f %f\n", i,
                          cSensor3DVector.GetXAngle().SignedNormalize().GetValue()*CRadians::RADIANS_TO_DEGREES,
                          cSensor3DVector.GetYAngle().SignedNormalize().GetValue()*CRadians::RADIANS_TO_DEGREES,
                          cSensor3DVector.GetZAngle().SignedNormalize().GetValue()*CRadians::RADIANS_TO_DEGREES);
                    DEBUG("Sensor to light direction: %f %f %f\n",
                          cSensorToLight.GetXAngle().SignedNormalize().GetValue()*CRadians::RADIANS_TO_DEGREES,
                          cSensorToLight.GetYAngle().SignedNormalize().GetValue()*CRadians::RADIANS_TO_DEGREES,
                          cSensorToLight.GetZAngle().SignedNormalize().GetValue()*CRadians::RADIANS_TO_DEGREES);
                    */
                            Real fAngle = (cSensorToLight.GetZAngle().SignedNormalize().GetValue() - cSensor3DVector.GetZAngle().SignedNormalize().GetValue())*CRadians::RADIANS_TO_DEGREES;
                            //Real fAngle = (cScanningVector.Angle().GetValue() - cSensorVector.Angle().GetValue())*CRadians::RADIANS_TO_DEGREES;
                            //DEBUG("Angle: %f\n", fAngle);
                            CRadians cTanAngle;
                            cTanAngle.FromValueInDegrees(90.0);
                            if (fAngle > -90 && fAngle < 90) {
                                m_tReadings[i] += CalculateReading(cSensorToLight.Length(),
                                                                   cLight.GetIntensity());
                            }
                        }
                    }
                }
            }
            /* Apply noise to the sensor */
            if(m_bAddNoise) {
                m_tReadings[i] += m_pcRNG->Uniform(m_cNoiseRange);
            }
            /* Trunc the reading between 0 and 1 */
            UNIT.TruncValue(m_tReadings[i]);
        }
    }else{
        /* There are no lights in the environment */
        if(m_bAddNoise) {
            /* Go through the sensors */
            for(UInt32 i = 0; i < m_tReadings.size(); ++i) {
                /* Apply noise to the sensor */
                m_tReadings[i] += m_pcRNG->Uniform(m_cNoiseRange);
                /* Trunc the reading between 0 and 1 */
                UNIT.TruncValue(m_tReadings[i]);
            }
        }
    }


    m_cReading = CLightVirtualSensorReading(m_tReadings);

    m_cVirtualSensorData.AppendVirtualSensorData(m_cReading, m_pairITSAndRobotId.second, m_unSensorId);
}

/****************************************/
/****************************************/

REGISTER_SENSOR(CLightVirtualSensor,
                "virtual_light_sensor", "default",
                "Mattia Salvaro",
                "1.0",
                "The general robot virtual light sensor.",
                "This sensor accesses the general virtual light sensor.\n",
                "Usable"
                );

}
