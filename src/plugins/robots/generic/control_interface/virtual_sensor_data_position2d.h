/**
 * @file <argos3/plugins/robots/generic/control_interface/virtual_sensor_data_position2d.h>
 *
 * @brief Implement the class CVirtualSensorNetworkData for Destination and GPS sensor
 * It represents a position in 2D (X,Y,rotation). This cass allow to serialize/deserialize data
 * to be send through the network by the virtual sensors.
 *
 * @author Bernard Mayeur[bmayeur@ulb.ac.be]
 */

#ifndef VIRTUAL_SENSOR_DATA_POSITION2D_H_
#define VIRTUAL_SENSOR_DATA_POSITION2D_H_

#include <vector>
//#include <math.h>
#include <argos3/plugins/robots/generic/control_interface/virtual_sensor_network_data.h>
//#include <argos3/core/utility/logging/argos_log.h>
//#include <stdlib.h>

namespace argos {

class CVirtualSensorDataPosition2D : virtual public CVirtualSensorNetworkData {
public:
	CVirtualSensorDataPosition2D() :
		XRange(0.0),
		YRange(0.0),
		Bearing(0.0){}

	CVirtualSensorDataPosition2D(Real f_x, Real f_y, Real f_rot):
		XRange(f_x),
		YRange(f_y),
		Bearing(f_rot){}

	/**
	 * Constructor of a packet from another one
	 */
	CVirtualSensorDataPosition2D(const CVirtualSensorDataPosition2D & t_packet):
		XRange(t_packet.XRange),
		YRange(t_packet.YRange),
		Bearing(t_packet.Bearing) {}

	/**
	 * Redefine the "=" operator
	 */
	virtual CVirtualSensorDataPosition2D& operator=(const CVirtualSensorDataPosition2D & t_packet) {
	   if (&t_packet != this) {
		  XRange = t_packet.XRange;
		  YRange = t_packet.YRange;
		  Bearing = t_packet.Bearing;
	   }
	   return *this;
	}

	/**
	 * Redefine the "==" operator
	 */
	virtual bool operator==(const CVirtualSensorDataPosition2D & t_packet) {
	   return     XRange == t_packet.XRange
			   && YRange == t_packet.YRange
			   && Bearing == t_packet.Bearing;
	}

	/**
		 * Redefine the "!=" operator
		 */
	virtual bool operator!=(const CVirtualSensorDataPosition2D & t_packet) {
	   return ! this->operator==(t_packet);
	}

	/*
	 * Transform the content of the class to a vector of char/UInt8
	 */
	virtual CByteArray Serialize() const{
		CByteArray vecBuffer ;
		vecBuffer << XRange << YRange << Bearing;
		return vecBuffer;
	}

	/*
	 * Change the values of the class to apply the content of the vector
	 */
	virtual void Deserialize(CByteArray& vec_buffer){
		vec_buffer >> XRange >> YRange >> Bearing;
	}

	virtual bool IsColliding(CVirtualSensorDataPosition2D cOther, Real fRadius){
		if ( ( (cOther.XRange - XRange) < fRadius*2) && ( (XRange - cOther.XRange) < fRadius*2 ) ){
			if ( ( (cOther.YRange - YRange) < fRadius*2) && ( (YRange - cOther.YRange) < fRadius*2 ) ){
				return true;
			}
		}
		return false;
	}

	Real XRange;
	Real YRange;
	Real Bearing;
};

}
#endif

