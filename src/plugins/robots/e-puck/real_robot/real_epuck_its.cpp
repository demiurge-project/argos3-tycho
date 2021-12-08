#include "real_epuck_its.h"

namespace argos {

/****************************************/
/****************************************/

CRealEPuckITS& CRealEPuckITS::GetInstance() {
   static std::auto_ptr<CRealEPuckITS> pcRealEPuckInstance(new CRealEPuckITS);
   return *(pcRealEPuckInstance.get());
}

/****************************************/
/****************************************/

CRealEPuckITS::CRealEPuckITS() :
	CRealEPuck(),
	m_cVirtualSensorClient(CVirtualSensorClient::GetInstance()),
    m_unVirtualSensorServerPort(0) {
}

/****************************************/
/****************************************/

CRealEPuckITS::~CRealEPuckITS()
{
    LOG << "[INFO] Destroying ITS client" << std::endl;
    try {
        m_cVirtualSensorClient.Destroy();
    }
    catch (CARGoSException cEx){
        LOGERR << "Exception thrown in Destroy\n";
        LOGERR << cEx.what() << "\n";
    }
    LOG << "[INFO] Destruction ITS completed" << std::endl;
}

void CRealEPuckITS::Init(const std::string& str_config_file_name,
                         const std::string& str_controller_id) {
    try {
        CRealEPuck::Init(str_config_file_name, str_controller_id);
        TConfigurationNode& tRoot = GetConfigurationRoot();
        TConfigurationNode& tFramework = GetNode(tRoot, "framework");
        TConfigurationNode& tExperiment = GetNode(tFramework, "experiment");
        GetNodeAttribute(tExperiment, "vss_host", m_strVirtualSensorServerAddress);
        GetNodeAttribute(tExperiment, "vss_port", m_unVirtualSensorServerPort);
        m_cVirtualSensorClient.SetServerAddresAndPort(m_strVirtualSensorServerAddress, m_unVirtualSensorServerPort);
    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing the real epuck", ex);
    }
}

/****************************************/
/****************************************/

/*
 *   Creates all the Virtual Sensors defined ad-hoc by the user in the xml
 *   configuration file for the ITS
 */
CCI_Sensor* CRealEPuckITS::InsertSensor(const std::string& str_sensor_name) {
    if (str_sensor_name == "virtual_ground_sensor") {
        CRealVirtualGroundSensor* pcVirtualGroundSensor = CreateSensor<CRealVirtualGroundSensor>(str_sensor_name);
        m_vecVirtualSensors.push_back(pcVirtualGroundSensor);
        return pcVirtualGroundSensor;
    }

    if (str_sensor_name == "virtual_light_sensor") {
        CRealVirtualLightSensor* pcVirtualLightSensor = CreateSensor<CRealVirtualLightSensor>(str_sensor_name);
        m_vecVirtualSensors.push_back(pcVirtualLightSensor);
        return pcVirtualLightSensor;
    }
    if (str_sensor_name == "gps_virtual_sensor") {
        CRealGPSVirtualSensor* pcGPSVirtualSensor = CreateSensor<CRealGPSVirtualSensor>(str_sensor_name);
        m_vecVirtualSensors.push_back(pcGPSVirtualSensor);
        return pcGPSVirtualSensor;
    }

    if (str_sensor_name == "destination_virtual_sensor") {
        CRealDestinationVirtualSensor* pcDestinationVirtualSensor = CreateSensor<CRealDestinationVirtualSensor>(str_sensor_name);
        m_vecVirtualSensors.push_back(pcDestinationVirtualSensor);
        return pcDestinationVirtualSensor;
    }
    if (str_sensor_name == "virtual_rgb_ground_sensor") {
        CRealVirtualRGBGroundSensor* pcVirtualRGBGroundSensor = CreateSensor<CRealVirtualRGBGroundSensor>(str_sensor_name);
        m_vecVirtualSensors.push_back(pcVirtualRGBGroundSensor);
        return pcVirtualRGBGroundSensor;
    }
    if (str_sensor_name == "virtual_radiation_sensor") {
        CRealVirtualRadiationSensor* pcVirtualRadiationSensor = CreateSensor<CRealVirtualRadiationSensor>(str_sensor_name);
        m_vecVirtualSensors.push_back(pcVirtualRadiationSensor);
        return pcVirtualRadiationSensor;
    }
    if (str_sensor_name == "virtual_pollutant_sensor") {
        CRealVirtualPollutantSensor* pcVirtualPollutantSensor = CreateSensor<CRealVirtualPollutantSensor>(str_sensor_name);
        m_vecVirtualSensors.push_back(pcVirtualPollutantSensor);
        return pcVirtualPollutantSensor;
    }
    if (str_sensor_name == "virtual_black_white_location_sensor") {
        CRealVirtualBlackWhiteLocationSensor* pcVirtualBlackWhiteLocationSensor = CreateSensor<CRealVirtualBlackWhiteLocationSensor>(str_sensor_name);
        m_vecVirtualSensors.push_back(pcVirtualBlackWhiteLocationSensor);
        return pcVirtualBlackWhiteLocationSensor;
    }


    else {
        return CRealEPuck::InsertSensor(str_sensor_name);
    }
}

/****************************************/
/****************************************/

void CRealEPuckITS::UpdateValues() {
   CRealEPuck::UpdateValues();
   // Update Virtual Sensors also
   UpdateVirtualSensors();
}

/****************************************/
/****************************************/

void CRealEPuckITS::UpdateVirtualSensors() {
   //DEBUG("m_vecVirtualSensors.size() = %d\n", m_vecVirtualSensors.size());
   for(size_t i = 0; i < m_vecVirtualSensors.size(); ++i) {
      m_vecVirtualSensors[i]->UpdateValues();
   }
}


}
