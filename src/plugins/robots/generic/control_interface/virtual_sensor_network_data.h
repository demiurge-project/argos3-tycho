/**
 * @file <argos3/plugins/robots/generic/control_interface/virtual_sensor_network_data.h>
 *
 * @brief This file provides an abstract class that contains all serialization functions.
 * Every virtual sensor's data structure that should be sent through the virtual sensor,
 * should implement this class
 *
 * @author Bernard Mayeur[bmayeur@ulb.ac.be]
 */

#ifndef NETWORK_DATA_H_
#define NETWORK_DATA_H_

#include <vector>
//#include <math.h>
#include <argos3/core/utility/datatypes/datatypes.h>
//#include <argos3/core/utility/datatypes/network_byte_array.h>
#include <argos3/core/utility/datatypes/byte_array.h>
//#include <argos3/core/utility/logging/argos_log.h>
//#include <stdlib.h>

namespace argos {

/*
 * Class used to define structure that should be send through the network
 * It uses the CNetworkByteArray to serialize/deserialize the content.
 */
class CVirtualSensorNetworkData {
public:
	CVirtualSensorNetworkData(){};
	virtual ~CVirtualSensorNetworkData(){};

	/**
	 * Constructor of a packet from another one
	 */
	CVirtualSensorNetworkData(const CVirtualSensorNetworkData & t_packet){};

	/**
	 * Constructor of a packet from a serialized packet
	 *
	 **** unused constructor generating a lot of warnings because you should
	 **** not call a virtual method from a constructor !!
	 */
	//CVirtualSensorNetworkData(CByteArray& vec_buffer){
		//this->Deserialize(vec_buffer);
	//}

	/*
	 * Transform the content of the class to a vector of char/UInt8
	 */
	virtual CByteArray Serialize() const = 0;
	/*
	 * Change the values of the class to apply the content of the vector
	 */
	virtual void Deserialize(CByteArray& vec_buffer) = 0;
};

}

#endif /* NETWORK_DATA_H_ */
