/*
 * pathPlanning.cpp
 *
 *  Created on: 27 juin 2014
 *      Author: bernard
 */

/* Include the controller definition */
#include "path_planning_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/

CPathPlanningController::CPathPlanningController() :
    m_pcWheelsActuator(NULL),
    m_pcProximitySensor(NULL),
    m_pcGPSSensor(NULL),
    m_pcDestinationSensor(NULL),
    m_pcLEDActuator(NULL),
    m_pcBatterySensor(NULL),
    m_fStopThreshold(0.03),
    m_fProximityThreshold(0.4),
    m_fProximityRepulsionForce(8),
    m_fProximityRepulsionForceNoise(4),
    m_fAngleStopThreshold(1),
    m_fBatteryTimeStepThreshold(10),
    m_unBatteryLowCounter(0),
    m_bForceBatteryChange(false),
    m_cRechargeDest(CVirtualSensorDataPosition2D(0,0,0)),
    m_unMaxTickForceRandomWalk(0),
    m_pcRNG(NULL)
{}

/****************************************/
/****************************************/

void CPathPlanningController::Init(TConfigurationNode& t_node) {
    try {
        m_pcWheelsActuator = GetActuator<CCI_EPuckWheelsActuator>("epuck_wheels");
    } catch (CARGoSException &ex) {}
    try {
    	m_pcGPSSensor = GetSensor<CCI_GPSVirtualSensor>("gps_virtual_sensor");
    } catch (CARGoSException &ex) {}
    try {
	    m_pcDestinationSensor = GetSensor<CCI_DestinationVirtualSensor>("destination_virtual_sensor");
	} catch (CARGoSException &ex) {}
	try {
		m_pcProximitySensor = GetSensor<CCI_EPuckProximitySensor>("epuck_proximity");
	} catch (CARGoSException &ex) {}
	try {
		m_pcLEDActuator = GetActuator<CCI_EPuckBaseLEDsActuator>("epuck_base_leds");
	} catch (CARGoSException &ex) {}
	try {
		m_pcRGBLEDActuator = GetActuator<CCI_EPuckRGBLEDsActuator>("epuck_rgb_leds");
	} catch (CARGoSException &ex) {}
	try {
		m_pcBatterySensor = GetSensor<CCI_EPuckBatterySensor>("epuck_battery");
	} catch (CARGoSException &ex) {}
    GetNodeAttributeOrDefault(t_node, "placementPrecision", m_fStopThreshold, m_fStopThreshold);
    GetNodeAttributeOrDefault(t_node, "proximitySignificativeThreshold", m_fProximityThreshold, m_fProximityThreshold);
    GetNodeAttributeOrDefault(t_node, "proximityRepulsionForce", m_fProximityRepulsionForce, m_fProximityRepulsionForce);
    GetNodeAttributeOrDefault(t_node, "proximityRepulsionForceNoise", m_fProximityRepulsionForceNoise, m_fProximityRepulsionForceNoise);
    GetNodeAttributeOrDefault(t_node, "anglePrecision", m_fAngleStopThreshold, m_fAngleStopThreshold);
    GetNodeAttributeOrDefault(t_node, "forceRandomWalkAfterNoUpdateFor", m_unMaxTickForceRandomWalk, m_unMaxTickForceRandomWalk);
    if (m_pcBatterySensor){
    	GetNodeAttributeOrDefault(t_node, "batteryTimeStepThreshold", m_fBatteryTimeStepThreshold, m_fBatteryTimeStepThreshold);
    	TConfigurationNode& tRechargeDestination = GetNode(t_node, "recharge_destination");
    	GetNodeAttribute(tRechargeDestination, "x", m_cRechargeDest.XRange);
    	GetNodeAttribute(tRechargeDestination, "y", m_cRechargeDest.YRange);
    	GetNodeAttribute(tRechargeDestination, "rot", m_cRechargeDest.Bearing);
    	DEBUG("Recharge_destination : %f %f %f\n",m_cRechargeDest.XRange,m_cRechargeDest.YRange,m_cRechargeDest.Bearing);
    }

    CRadians cAngleThreshold;
    cAngleThreshold.FromValueInDegrees(m_fAngleStopThreshold);
    m_fAngleStopThreshold = cAngleThreshold.SignedNormalize().GetAbsoluteValue();
    m_pcRNG= argos::CRandom::CreateRNG("argos");
}



/****************************************/
/****************************************/

void CPathPlanningController::ControlStep() {
  m_pcRGBLEDActuator->SetColors(CColor::BLACK);
	// No pathplanning possible if no GPS/Destination sensor
	if (m_pcDestinationSensor && m_pcGPSSensor)
	{
		CVirtualSensorDataPosition2D cPos = m_pcGPSSensor->GetReading();
		CVirtualSensorDataPosition2D cDest = m_pcDestinationSensor->GetReading();

    if (cDest.XRange != 0 && cDest.YRange != 0) {
  		CVector2 cPathVect (-cPos.XRange + cDest.XRange,-cPos.YRange + cDest.YRange);
  		cPathVect= cPathVect.Rotate(CRadians(-cPos.Bearing));

  		m_pcLEDActuator->SwitchLEDs(false);
  		/* If battery sensor, verify the level of the battery */
  		if (m_pcBatterySensor && !m_pcBatterySensor->GetBatteryLevel() && cPathVect.Length()>m_fStopThreshold){
  			++m_unBatteryLowCounter;
  			if (m_unBatteryLowCounter > m_fBatteryTimeStepThreshold)
  				m_bForceBatteryChange = true;
  		}
  		else if (m_unBatteryLowCounter < m_fBatteryTimeStepThreshold)
  			m_unBatteryLowCounter=0;
  		/* If we want to force to change the battery, go to the recharge destination */
  		if (m_bForceBatteryChange) {
  			cPathVect = CVector2 (-cPos.XRange + m_cRechargeDest.XRange,-cPos.YRange + m_cRechargeDest.YRange);
  			cPathVect = cPathVect.Rotate(CRadians(-cPos.Bearing));
  			if (cPathVect.Length()>0.05){
  				Real fSpeed = 11.0/(1+pow(2.718281828459045,-30*cPathVect.Length()+4)) +3;
  				CVector2 cProximity = ParseProximity();
  				if (cProximity.Length()>m_fProximityThreshold){
  					Real fRepulsion = m_pcRNG->Uniform(CRange<Real>(m_fProximityRepulsionForce-m_fProximityRepulsionForceNoise, m_fProximityRepulsionForce+m_fProximityRepulsionForceNoise));
  					followVector(cPathVect.Normalize()-fRepulsion*cProximity.Normalize(), fSpeed);
  					m_pcLEDActuator->SwitchLEDs(true);
            //LOG<<" Low bat 3 \n";
  				}
  				else{
  					followVector(cPathVect, fSpeed);
  					m_pcLEDActuator->SwitchLED(m_unBatteryLowCounter%8,true);
  					m_pcLEDActuator->SwitchLED((m_unBatteryLowCounter+4)%8,true);
            //LOG<<" Low bat 1 \n";
  				}
  			}
  			else{
  				m_pcWheelsActuator->SetLinearVelocity(0,0);
  				m_pcLEDActuator->SwitchLED(m_unBatteryLowCounter%8,true);
  				m_pcLEDActuator->SwitchLED((m_unBatteryLowCounter+4)%8,true);
          //LOG<<" Low bat 2 \n";
  				//Terminate the receiving thread and then the controller
  				CVirtualSensorClient::GetInstance().ForceEndOfConnection();
  			}
  			/*DEBUG("Recharge destination:\n");
  			DEBUG("X : %f\n",m_cRechargeDest.XRange);
  			DEBUG("Y : %f\n",m_cRechargeDest.YRange);
  			DEBUG("\n\n\n");*/
  		}
  		/* Else, if in position, correct the angle */
  		else if (cPathVect.Length()<=m_fStopThreshold){
  			CRadians fPos(cPos.Bearing);
  			CRadians fDest(cDest.Bearing);
  			if ((fPos-fDest).SignedNormalize().GetAbsoluteValue() > m_fAngleStopThreshold) {
          m_pcRGBLEDActuator->SetColor(2, CColor::BROWN);
  				SetLinearRotation(fPos,fDest);
  			} else {
          m_pcRGBLEDActuator->SetColor(2, CColor::GREEN);
  				m_pcWheelsActuator->SetLinearVelocity(0,0);
        }
  		}
  		/* else if we are not in position and position have been updated, we go to destination */
  		else if (/*cPathVect.Length()>m_fStopThreshold && */(!m_unMaxTickForceRandomWalk || (m_unMaxTickForceRandomWalk && m_pcGPSSensor->GetTickTimeTillLastUpdate() < m_unMaxTickForceRandomWalk))){
  				Real fSpeed = 11.0/(1+pow(2.718281828459045,-30*cPathVect.Length()+4)) +3;
  				CVector2 cProximity = ParseProximity();
          //LOG<<"prox lenght: "<<cProximity.Length()<<std::endl;
  				if (cProximity.Length()>m_fProximityThreshold){
  					Real fRepulsion = m_pcRNG->Uniform(CRange<Real>(m_fProximityRepulsionForce-m_fProximityRepulsionForceNoise, m_fProximityRepulsionForce+m_fProximityRepulsionForceNoise));
  					followVector(cPathVect.Normalize()-fRepulsion*cProximity.Normalize(), fSpeed);
  					m_pcLEDActuator->SwitchLEDs(true);
            //LOG<<" Proximity detected 1 \n";
  				}
  				else
  					followVector(cPathVect, fSpeed);
  			}
  		/* else position not updated, random walk */
  		else {
  			// GPS not updated for long, tag not seen anymore, should random walk to find it back
        m_pcRGBLEDActuator->SetColor(0, CColor::ORANGE);
  			cPathVect = CVector2(1,0); // Go straight
  			CVector2 cProximity = ParseProximity();
  			if (cProximity.Length()>m_fProximityThreshold){
  				Real fRepulsion = m_pcRNG->Uniform(CRange<Real>(m_fProximityRepulsionForce-m_fProximityRepulsionForceNoise, m_fProximityRepulsionForce+m_fProximityRepulsionForceNoise));
  				followVector(cPathVect-fRepulsion*cProximity.Normalize(), 14);
  			}
  			else
  				followVector(cPathVect, 14);
  			if (m_pcGPSSensor->GetTickTimeTillLastUpdate()%10 == 0)
  				m_pcLEDActuator->SwitchLEDs(true);
          //LOG<<"GPS not updated for long 2 \n";
  		}
    }
	}
	else
		THROW_ARGOSEXCEPTION("Virtual GPS & Destination needed for pathPlanning");
}

void CPathPlanningController::followVector(CVector2 sum, Real f_speed) {
	/*
	 * We follow the vector by using curves instead of lines to minimize randomness and lag influance
	 */
    CRadians norm = sum.Angle().UnsignedNormalize();
    CRange<CRadians> left1(CRadians::PI, CRadians::TWO_PI);
    CRange<CRadians> right1(CRadians::ZERO, CRadians::PI);
    Real right = 0;
    Real left = 0;

    if (sum.GetX() != 0 || sum.GetY() != 0) {
        if (right1.WithinMinBoundExcludedMaxBoundExcluded(norm)) {
            right = 1;
        } else {
            right = Max<Real > (-1.0f, Cos(norm));
        }
        if (left1.WithinMinBoundExcludedMaxBoundExcluded(norm)) {
            left = 1;
        } else {
            left = Max<Real > (-1.0f, Cos(norm));
        }
    }
    if (std::abs(left) + std::abs(right) == 0.0f) {
        m_pcWheelsActuator->SetLinearVelocity(0, 0);
    } else {
        Real p = f_speed / std::max(std::abs(left), std::abs(right));
        Real l = p * left;
        Real r = p * right;
        m_pcWheelsActuator->SetLinearVelocity(l, r);
    }
}

void CPathPlanningController::followVectorStraight(CVector2 c_vector_to_follow, Real f_speed) {
  // Method taken from AutoMoDe
  Real fLeftVelocity = 0;
  Real fRightVelocity = 0;
  CRange<CRadians> cLeftHemisphere(CRadians::ZERO, CRadians::PI);
  CRange<CRadians> cRightHemisphere(CRadians::PI, CRadians::TWO_PI);
  CRadians cNormalizedVectorToFollow = c_vector_to_follow.Angle().UnsignedNormalize();
  if (c_vector_to_follow.GetX() != 0 || c_vector_to_follow.GetY() != 0) {
    if (cLeftHemisphere.WithinMinBoundExcludedMaxBoundExcluded(cNormalizedVectorToFollow)) {
      fRightVelocity = 1;
      fLeftVelocity = Cos(cNormalizedVectorToFollow);
    } else {
      fRightVelocity = Cos(cNormalizedVectorToFollow);
      fLeftVelocity = 1;
    }
  }
  Real fVelocityFactor = f_speed / Max<Real>(std::abs(fRightVelocity), std::abs(fLeftVelocity));
  m_pcWheelsActuator->SetLinearVelocity(fVelocityFactor * fLeftVelocity, fVelocityFactor * fRightVelocity);
}


void CPathPlanningController::SetLinearRotation(CRadians fPos,CRadians fDest){
	Real fSpeed = (fPos-fDest).SignedNormalize().GetValue()*2;
	m_pcWheelsActuator->SetLinearVelocity(fSpeed, -fSpeed);
}


CVector2 CPathPlanningController::ParseProximity() {
	if (m_pcProximitySensor){
		CCI_EPuckProximitySensor::TReadings tReadings = m_pcProximitySensor->GetReadings();
		CVector2 sum(0, 0);
		for (UInt32 i = 0; i < tReadings.size(); i++) {
			sum += CVector2(tReadings[i].Value, tReadings[i].Angle);
		}
		return sum;
	}
	return CVector2();
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
REGISTER_CONTROLLER(CPathPlanningController, "path_planning_controller");
