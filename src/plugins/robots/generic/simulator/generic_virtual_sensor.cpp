#include "generic_virtual_sensor.h"

namespace argos {

/****************************************/
/****************************************/

CGenericVirtualSensor::CGenericVirtualSensor()
    : m_pairITSAndRobotId(),
      m_cVirtualSensorData(CVirtualSensorData::GetInstance()),
      m_unVirtualSensorDataSize(0),
      m_bRealExperiment(false),
      m_unSensorId(0)
{}

/****************************************/
/****************************************/

void CGenericVirtualSensor::GetITSAndRobotIdFromArgosId(std::pair<UInt32, UInt32> &pair_its_androbot_id, const std::string &str_argos_id)
{
    std::vector<std::string> vecTokens;
    std::stringstream cStringStream(str_argos_id);
    std::string strToken;
    // Argos ID must be structured as <string>_TAG_IP
    while (std::getline(cStringStream, strToken, '_')) {
        vecTokens.push_back(strToken);
    }

    UInt32 unITSId;
    UInt32 unRobotId;

    try {
        unITSId = FromString<UInt32>(vecTokens[vecTokens.size()-2]);
        unRobotId = FromString<UInt32>(vecTokens[vecTokens.size()-1]);
    }
    catch(std::exception& cEx) {
        LOGERR << "ERROR IN ROBOTS NAME. Rename the robots in the form <string>_TAG_IP\n";
        LOGERR << cEx.what() << "\n";
        THROW_ARGOSEXCEPTION("ERROR IN ROBOTS NAME. Rename the robots in the form <string>_TAG_IP");
    }


    //pair_its_androbot_id = std::make_pair<UInt32, UInt32>(unITSId, unRobotId);
    pair_its_androbot_id = std::make_pair(unITSId, unRobotId);
}

}
