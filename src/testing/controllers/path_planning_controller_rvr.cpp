/*
 * Author: Miquel Kegeleirs
 */

/* Include the controller definition */
#include "path_planning_controller_rvr.h"

/****************************************/
/****************************************/

CPathPlanningControllerRVR::CPathPlanningControllerRVR() :
    m_pcWheelsActuator(NULL),
	m_pcVelocitySensor(NULL),
    m_pcProximitySensor(NULL),
    m_pcGPSSensor(NULL),
    m_pcDestinationSensor(NULL),
    m_pcLEDsActuator(NULL),
    m_fStopThreshold(0.03),
    m_fProximityThreshold(0.4),
    m_fProximityRepulsionForce(8),
    m_fProximityRepulsionForceNoise(4),
    m_fAngleStopThreshold(1),
    m_unMaxTickForceRandomWalk(0),
	m_fDefaultWheelVelocity(15.55f),
    m_pcRNG(NULL)
{}

/****************************************/
/****************************************/

void CPathPlanningControllerRVR::Init(TConfigurationNode& t_node) 
{
    try {
        m_pcWheelsActuator = GetActuator<CCI_RVRWheelsActuator>("rvr_wheels");
    } catch (CARGoSException &ex) {}
	try {
        m_pcVelocitySensor = GetSensor<CCI_RVRVelocitySensor>("rvr_velocity");
	} catch (CARGoSException &ex) {}
	try {
        m_pcProximitySensor = GetSensor<CCI_RVRProximitySensor>("rvr_proximity");
	} catch (CARGoSException &ex) {}
    try {
    	m_pcGPSSensor = GetSensor<CCI_GPSVirtualSensor>("gps_virtual_sensor");
    } catch (CARGoSException &ex) {}
    try {
	    m_pcDestinationSensor = GetSensor<CCI_DestinationVirtualSensor>("destination_virtual_sensor");
	} catch (CARGoSException &ex) {}
	try {
        m_pcLEDsActuator = GetActuator<CCI_RVRRGBLEDsActuator>("rvr_rgb_leds");
	} catch (CARGoSException &ex) {}
	
    GetNodeAttributeOrDefault(t_node, "placementPrecision", m_fStopThreshold, m_fStopThreshold);
    GetNodeAttributeOrDefault(t_node, "proximitySignificativeThreshold", m_fProximityThreshold, m_fProximityThreshold);
    GetNodeAttributeOrDefault(t_node, "proximityRepulsionForce", m_fProximityRepulsionForce, m_fProximityRepulsionForce);
    GetNodeAttributeOrDefault(t_node, "proximityRepulsionForceNoise", m_fProximityRepulsionForceNoise, m_fProximityRepulsionForceNoise);
    GetNodeAttributeOrDefault(t_node, "anglePrecision", m_fAngleStopThreshold, m_fAngleStopThreshold);
    GetNodeAttributeOrDefault(t_node, "forceRandomWalkAfterNoUpdateFor", m_unMaxTickForceRandomWalk, m_unMaxTickForceRandomWalk);
    GetNodeAttributeOrDefault(t_node, "velocity", m_fDefaultWheelVelocity, m_fDefaultWheelVelocity);
    leftWheelVelocity = m_fDefaultWheelVelocity;
    rightWheelVelocity = m_fDefaultWheelVelocity;

    CRadians cAngleThreshold;
    cAngleThreshold.FromValueInDegrees(m_fAngleStopThreshold);
    m_fAngleStopThreshold = cAngleThreshold.SignedNormalize().GetAbsoluteValue();
    m_pcRNG= argos::CRandom::CreateRNG("argos");

	InitRos();
}

/****************************************/
/****************************************/

void CPathPlanningControllerRVR::InitRos()
{
    std::stringstream name;
    name.str("");
    name << GetId();

    if (!ros::isInitialized())
    {
        char **argv = NULL;
        int argc = 0;
        ros::init(argc, argv, name.str());
    }
    ros::NodeHandle rosNode;

    // setup teraranger subscriber
    prox_sub = rosNode.subscribe("ranges", 10, &CPathPlanningControllerRVR::TerarangerHandler, this);

    // setup velocity publisher
    vel_pub = rosNode.advertise<std_msgs::Float32MultiArray>("/rvr/wheels_speed", 10, true);
    // setup velocity messages
    vel_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    vel_msg.layout.dim[0].size = 2;
    vel_msg.layout.dim[0].stride = 1;
    vel_msg.layout.dim[0].label = "wheel_vel";
    vel_msg.data.clear();

    // setup LED publisher
    led_pub = rosNode.advertise<rvr_ros::Leds>("/rvr/rgb_leds", 10, true);
}

/****************************************/
/****************************************/

void CPathPlanningControllerRVR::ControlStep() 
{
  	SetColors(CColor::BLACK);
	// No pathplanning possible if no GPS/Destination sensor
	if (m_pcDestinationSensor && m_pcGPSSensor)
	{
		CVirtualSensorDataPosition2D cPos = m_pcGPSSensor->GetReading();
		CVirtualSensorDataPosition2D cDest = m_pcDestinationSensor->GetReading();

		if (cDest.XRange != 0 && cDest.YRange != 0) 
		{
			CVector2 cPathVect (-cPos.XRange + cDest.XRange,-cPos.YRange + cDest.YRange);
			cPathVect= cPathVect.Rotate(CRadians(-cPos.Bearing));
			/* If in position, correct the angle */
			if (cPathVect.Length()<=m_fStopThreshold)
			{
				CRadians fPos(cPos.Bearing);
				CRadians fDest(cDest.Bearing);
				if ((fPos-fDest).SignedNormalize().GetAbsoluteValue() > m_fAngleStopThreshold) 
				{
					SetColors(CColor::BROWN);
					SetLinearRotation(fPos,fDest);
				} 
				else 
				{
					SetColors(CColor::GREEN);
					SetVelocity(0,0);
				}
			}
			/* else if we are not in position and position have been updated, we go to destination */
			else if (/*cPathVect.Length()>m_fStopThreshold && */(!m_unMaxTickForceRandomWalk || (m_unMaxTickForceRandomWalk && m_pcGPSSensor->GetTickTimeTillLastUpdate() < m_unMaxTickForceRandomWalk)))
			{
				Real fSpeed = 11.0/(1+pow(2.718281828459045,-30*cPathVect.Length()+4)) +3;
				CVector2 cProximity = ParseProximity();
			//LOG<<"prox lenght: "<<cProximity.Length()<<std::endl;
				if (cProximity.Length()>m_fProximityThreshold)
				{
					Real fRepulsion = m_pcRNG->Uniform(CRange<Real>(m_fProximityRepulsionForce-m_fProximityRepulsionForceNoise, m_fProximityRepulsionForce+m_fProximityRepulsionForceNoise));
					followVector(cPathVect.Normalize()-fRepulsion*cProximity.Normalize(), fSpeed);
				//LOG<<" Proximity detected 1 \n";
				}
				else
				{
					followVector(cPathVect, fSpeed);
				}
			}
			/* else position not updated, random walk */
			else 
			{
				// GPS not updated for long, tag not seen anymore, should random walk to find it back
				SetColors(CColor::ORANGE);
				cPathVect = CVector2(1,0); // Go straight
				CVector2 cProximity = ParseProximity();
				if (cProximity.Length()>m_fProximityThreshold)
				{
					Real fRepulsion = m_pcRNG->Uniform(CRange<Real>(m_fProximityRepulsionForce-m_fProximityRepulsionForceNoise, m_fProximityRepulsionForce+m_fProximityRepulsionForceNoise));
					followVector(cPathVect-fRepulsion*cProximity.Normalize(), 14);
				}
				else
				{
					followVector(cPathVect, 14);
				}		
			//LOG<<"GPS not updated for long 2 \n";
			}
		}
	}
	else
	{
		THROW_ARGOSEXCEPTION("Virtual GPS & Destination needed for pathPlanning");
	}

    ros::spinOnce();
}

/****************************************/
/****************************************/

void CPathPlanningControllerRVR::followVector(CVector2 sum, Real f_speed) 
{
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
        SetVelocity(0, 0);
    } else {
        Real p = f_speed / std::max(std::abs(left), std::abs(right));
        Real l = p * left;
        Real r = p * right;
        SetVelocity(l, r);
    }
}

/****************************************/
/****************************************/

void CPathPlanningControllerRVR::followVectorStraight(CVector2 c_vector_to_follow, Real f_speed) {
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
  SetVelocity(fVelocityFactor * fLeftVelocity, fVelocityFactor * fRightVelocity);
}

/****************************************/
/****************************************/

void CPathPlanningControllerRVR::SetLinearRotation(CRadians fPos,CRadians fDest){
	Real fSpeed = (fPos-fDest).SignedNormalize().GetValue()*2;
	SetVelocity(fSpeed, -fSpeed);
}

/****************************************/
/****************************************/

CVector2 CPathPlanningControllerRVR::ParseProximity() {
	if (m_pcProximitySensor){
		CVector2 sum(0, 0);
		for (UInt32 i = 0; i < m_sProximityInput.size(); i++) {
			sum += CVector2(m_sProximityInput[i].Value, m_sProximityInput[i].Angle);
		}
		return sum;
	}
	return CVector2();
}

/****************************************/
/****************************************/

void CPathPlanningControllerRVR::TerarangerHandler(const teraranger_array::RangeArray &msg)
{
    for (size_t i = 0; i < 8; ++i)
    {
        if (msg.ranges[i].range <= 0.4f)
        {
            m_sProximityInput.at(i).Value = msg.ranges[i].range;
        }
        else
        {
            m_sProximityInput.at(i).Value = std::numeric_limits<Real>::max();
        }

        // CRange<Real>(0.0f, 1.0f).TruncValue(m_sProximityInput.at(i).Value);
    }
}

/****************************************/
/****************************************/

void CPathPlanningControllerRVR::SetColors(const CColor& color)
{
	for (int i = 0; i < 5; i++)
    {
        led_msg.led_colors[i].r = color.GetRed();
        led_msg.led_colors[i].g = color.GetGreen();
        led_msg.led_colors[i].b = color.GetBlue();
    }
    led_pub.publish(led_msg);
}

void CPathPlanningControllerRVR::SetVelocity(const Real& f_leftSpeed, const Real& f_rightspeed)
{
	leftWheelVelocity = f_leftSpeed;
	rightWheelVelocity = f_rightspeed;
	vel_msg.data.clear();
    vel_msg.data.push_back(round(leftWheelVelocity / 100)); // convert cm/s to m/s
    vel_msg.data.push_back(round(rightWheelVelocity / 100));
    vel_pub.publish(vel_msg);
}

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
REGISTER_CONTROLLER(CPathPlanningControllerRVR, "path_planning_controller_rvr");
