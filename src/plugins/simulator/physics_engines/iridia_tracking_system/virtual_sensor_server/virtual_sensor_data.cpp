/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/virtual_sensor_server/virtual_sensor_data.cpp>
 *
 *
 * @author Mattia Salvaro
 */

#include "virtual_sensor_data.h"

namespace argos {

/****************************************/
/****************************************/

CVirtualSensorData::CVirtualSensorData()
    : m_bVirtualSensorDefined(false)
{
    m_tVirtualSensorTable = new TVirtualSensorTable();
    m_tVirtualSensorData = new TVirtualSensorData();
}

/****************************************/
/****************************************/

CVirtualSensorData::~CVirtualSensorData()
{
    delete m_tVirtualSensorTable;
    delete m_tVirtualSensorData;
}

/****************************************/
/****************************************/

void CVirtualSensorData::AddVirtualSensorEntry(const UInt8 un_virtual_sensor_id, const UInt32 un_virtual_sensor_data_size)
{
    //m_tVirtualSensorTable->insert(std::make_pair<UInt8, UInt32>(un_virtual_sensor_id, un_virtual_sensor_data_size));
    m_tVirtualSensorTable->insert(std::make_pair(un_virtual_sensor_id, un_virtual_sensor_data_size));
    PrintVirtualSensorTable();
}

/****************************************/
/****************************************/

UInt32 CVirtualSensorData::GetDataSizeBySensorId(UInt8 un_sensor_id)
{
    TVirtualSensorTable::iterator itVirtualSensorTable = m_tVirtualSensorTable->find(un_sensor_id);
    if (itVirtualSensorTable != m_tVirtualSensorTable->end()) {
        return (*itVirtualSensorTable).second;
    }
    else {
        THROW_ARGOSEXCEPTION("Sensor not in VST\n");
    }
    return UInt32(0);
}

/****************************************/
/****************************************/

void CVirtualSensorData::AppendVirtualSensorData(const CVirtualSensorNetworkData& c_virtual_sensor_data, UInt32 un_robot_id, UInt8 un_sensor_id){
    TVirtualSensorData::iterator itVirtualSensorData = m_tVirtualSensorData->find(un_robot_id);

    // If Virtual Sensor Data does not contain an entry for the given robot, create one.
    if (itVirtualSensorData == m_tVirtualSensorData->end()) {
        LOG << "VST does not contain entry for robot: "<< un_robot_id<<"\n";
        SVirtualSensorData sVirtualSensorData = SVirtualSensorData();
        //m_tVirtualSensorData->insert(std::make_pair<UInt32, SVirtualSensorData>(un_robot_id, sVirtualSensorData));
        m_tVirtualSensorData->insert(std::make_pair(un_robot_id, sVirtualSensorData));
    }

    itVirtualSensorData = m_tVirtualSensorData->find(un_robot_id);
    // Set the given buffer in the given robot preparing buffer
    itVirtualSensorData->second.SetPreparingBuffer(c_virtual_sensor_data, un_sensor_id);
}

/****************************************/
/****************************************/

void CVirtualSensorData::SwapBuffers(UInt32 un_robot_id)
{
    TVirtualSensorData::iterator itVirtualSensorData = m_tVirtualSensorData->find(un_robot_id);
    // If the entry for the given robot exists, swap its buffers.
    if (itVirtualSensorData != m_tVirtualSensorData->end()) {
        (*itVirtualSensorData).second.SwitchBuffers();
    }
}

/****************************************/
/****************************************/

const CByteArray& CVirtualSensorData::GetReadyData(UInt32 un_robot_id)
{
    TVirtualSensorData::iterator itVirtualSensorData = m_tVirtualSensorData->find(un_robot_id);
    // If the entry for the given robot exists, get its ready readable data buffer.
    if (itVirtualSensorData == m_tVirtualSensorData->end()) {
    	THROW_ARGOSEXCEPTION("Asked ready data for not existing robot");
    }
    return itVirtualSensorData->second.GetReadyBuffer();
}

/****************************************/
/****************************************/

const CByteArray& CVirtualSensorData::GetPreparingData(UInt32 un_robot_id)
{
    TVirtualSensorData::iterator itVirtualSensorData = m_tVirtualSensorData->find(un_robot_id);
    // If the entry for the given robot exists, get its preparing writable data buffer.
    if (itVirtualSensorData == m_tVirtualSensorData->end()) {
    	THROW_ARGOSEXCEPTION("Asked preparing data for not existing robot");
    }
    return itVirtualSensorData->second.GetPreparingBuffer();
}

/****************************************/
/****************************************/

bool CVirtualSensorData::IsSensorAlreadyInTable(UInt8 un_sensor_id)
{
    //PrintVirtualSensorTable();
    TVirtualSensorTable::iterator itVirtualSensorTable = m_tVirtualSensorTable->find(un_sensor_id);
    // If the entry for the given sensor exists, return true.
    if(itVirtualSensorTable != m_tVirtualSensorTable->end()) {
        return true;
    }
    return false;
}

/****************************************/
/****************************************/

void CVirtualSensorData::PrintVirtualSensorTable()
{

    LOG<<"***************************\n";
    LOG<<"Virtual Sensor Table:\n";
    TVirtualSensorTable::iterator itVirtualSensorTable;
    for (itVirtualSensorTable = m_tVirtualSensorTable->begin(); itVirtualSensorTable != m_tVirtualSensorTable->end(); itVirtualSensorTable++)
    {
        LOG<<"\tSensor ID: "<<(*itVirtualSensorTable).first<< "\tData size: "<<(*itVirtualSensorTable).second<< "\n";
    }
    LOG<<"***************************\n";
}

/****************************************/
/****************************************/

const CByteArray* CVirtualSensorData::GetSerializedVirtualSensorTable()
{
	CByteArray* serialize = new CByteArray();

    TVirtualSensorTable::iterator itVirtualSensorTable;
    // For each sensor entry
    for (itVirtualSensorTable = m_tVirtualSensorTable->begin(); itVirtualSensorTable != m_tVirtualSensorTable->end(); itVirtualSensorTable++)
    {
        // Copy in the output buffer the Virtual Sensor ID and the Virtual Sensor data size
    	(*serialize) << itVirtualSensorTable->first << itVirtualSensorTable->second;
    }
    return serialize;
}

/****************************************/
/****************************************/

void CVirtualSensorData::SetVirtualSensorDefined()
{
    m_bVirtualSensorDefined = true;
}

/****************************************/
/****************************************/

bool CVirtualSensorData::IsAtLeastOneVirtualSensorDefined()
{
    return m_bVirtualSensorDefined;
}

/****************************************/
/****************************************/

}
