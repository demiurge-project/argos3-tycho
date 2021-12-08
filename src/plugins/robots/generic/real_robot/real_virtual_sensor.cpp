#include "real_virtual_sensor.h"

namespace argos {

CRealVirtualSensor::CRealVirtualSensor() :
	m_unSensorId(0),
    m_cVirtualSensorClient(CVirtualSensorClient::GetInstance()) {}

}
