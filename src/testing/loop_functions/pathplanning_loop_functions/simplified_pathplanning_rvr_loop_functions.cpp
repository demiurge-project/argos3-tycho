#include "simplified_pathplanning_rvr_loop_functions.h"
#include "position_generator.h"

/****************************************/
/****************************************/

CSimplifiedPathPlanningRVRLoopFunctions::CSimplifiedPathPlanningRVRLoopFunctions() :
    m_vecDestinations(),
    m_cAssignmentAlgorithm(NULL),
    m_cPathAlgorithm(NULL),
    m_strEntityType(""),
    m_fRobotBodyRadius(0.0),
    m_unSeed(0),
    m_unTotalNumberRobot(0),
    m_unInPosition(0),
    m_unTimeStepInPosition(20),
    //m_pcFloor(new CFloorEntity("loopFunctionFloor","src/testing/loop_functions/pathplanning_loop_functions/pathplanning.png")),
    m_pcExpFloor(NULL),
    m_bPathPlanning(true),
    m_bForceStopReset(true),
    m_bAllRobotsInPosition(false),
    m_bRandomDistribution(false),
    m_tickBudget(0),
    m_fBudgetPlacement(1),
    m_unBudgetPlacementTimer(0),
    m_unMaxBudgetPlacementTimer(0),
    m_pcExperimentLoopFunctions(NULL),
    m_pcPositionGenerator(NULL),
    m_pcOrientationGenerator(NULL),
    m_unMaxTrials(0),
    m_unStepCounter(0),
    m_bRobotsPerceived(false){
}

/****************************************/
/****************************************/

void CSimplifiedPathPlanningRVRLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      TConfigurationNode& tPathPlanning = GetNode(t_node, "pathplanning");
      GetNodeAttribute(tPathPlanning, "randomDistribution", m_bRandomDistribution);
      GetNodeAttribute(tPathPlanning, "robots", m_unTotalNumberRobot);
      GetNodeAttribute(tPathPlanning, "robotType", m_strEntityType);
      GetNodeAttributeOrDefault(tPathPlanning, "timestepInPosition", m_unTimeStepInPosition, m_unTimeStepInPosition);
      GetNodeAttributeOrDefault(tPathPlanning, "tickBudget", m_tickBudget, m_tickBudget);
      GetNodeAttributeOrDefault(tPathPlanning, "minProcentPlacementBudget", m_fBudgetPlacement, m_fBudgetPlacement);
      GetNodeAttributeOrDefault(tPathPlanning, "maxTimeAfterBudgetPlacement", m_unMaxBudgetPlacementTimer, m_unMaxBudgetPlacementTimer);

      /* INIT GRAPH AND ASSIGNMENT TYPE */
      std::string strParam = "min_max_assignment_generator";
      GetNodeAttributeOrDefault(tPathPlanning, "assigment", strParam, strParam);
      m_cAssignmentAlgorithm = CFactory<CAssignmentGenerator>::New(strParam);
      m_cAssignmentAlgorithm->Init(tPathPlanning);
      m_cPathAlgorithm = m_cAssignmentAlgorithm->GetPathGenerator();

      if (m_strEntityType == "rvr"){
    	  m_fRobotBodyRadius = 0.143f;
  	  }
  	  else
  		  THROW_ARGOSEXCEPTION("Unkown robot type");

      /* Verify that the number of robots defined in the argos configuration file corresponds to the number of robots indicated in the parameters */
      UInt8 unNumberRobotEntityDefined = 0;
      TConfigurationNodeIterator itArenaItem;
  	  for(itArenaItem = itArenaItem.begin(&GetNode(CSimulator::GetInstance().GetConfigurationRoot(), "arena")); itArenaItem != itArenaItem.end(); ++itArenaItem) {
  		  if(itArenaItem->Value() == m_strEntityType) {
  			  unNumberRobotEntityDefined++;
  		  }
  	  }
      if (unNumberRobotEntityDefined != m_unTotalNumberRobot){
  		  LOGERR << "Incorrect number of robots defined, " << unNumberRobotEntityDefined << " robots detected, " << m_unTotalNumberRobot << " needed of type " << m_strEntityType << std::endl;
  		  THROW_ARGOSEXCEPTION("Incorrect number of robots.");
  	  }

      /* Handling distributions parameters */
      if (!m_bRandomDistribution) {
        ParseFixedDestinations(tPathPlanning); // Destinations of robots are hard-coded in the experiment file
      }
      else {
    	  ParseRandomDistributionParameters(tPathPlanning); // Destinations of robots are to be randomly defined
      }

      /* Open file for time of experiments (appending to it) */
      try {
		    m_loopTime_list_file.open("loopTime.out", std::ios_base::app | std::ios_base::out);
		    if (!m_loopTime_list_file) {
			    LOGERR << "Can't open input file loopTime.out\n";
			    THROW_ARGOSEXCEPTION("Can't open input file  loopTime.out");
		    }
	    } catch(CARGoSException& ex) {}

   } catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing parameters!", ex);
   }

   m_pcExpFloor = &GetSpace().GetFloorEntity();
   //GetSpace().SetFloorEntity(*m_pcFloor);
}

/****************************************/
/****************************************/

void CSimplifiedPathPlanningRVRLoopFunctions::ParseFixedDestinations(TConfigurationNode&  tParametersNode) {
  /* Example of parameters for fixed destinations of 3 robots.
  <pathplanning randomDistribution="false" sharedDestination="true/false" robots="3" robotType="rvr">
    <destinations>
      <destination1 x="0.5" y="0.5" rot="-6.2" />
      <destination2 x="-0.5" y="0.5" rot="3.14" />
      <destination3 x="0" y="0.5" rot="1" />
    </destinations>
  </pathplanning> */
  UInt8 size;
  char destinationNumber[50];
  m_vecDestinations.clear();
  TConfigurationNode& tPathPlanningDestinationList = GetNode(tParametersNode, "destinations");
  // Read each destination
  for (UInt32 j=0; j < m_unTotalNumberRobot; ++j){
    size=sprintf(destinationNumber, "destination%d", j+1);
    if (size < 12)
      THROW_ARGOSEXCEPTION("ERROR DURING PARSING OF DESTINATION NODES")
    TConfigurationNode& tPathPlanningDestination = GetNode(tPathPlanningDestinationList, destinationNumber);
    CVirtualSensorDataPosition2D cDest;
    GetNodeAttribute(tPathPlanningDestination, "x", cDest.XRange);
    GetNodeAttribute(tPathPlanningDestination, "y", cDest.YRange);
    GetNodeAttribute(tPathPlanningDestination, "rot", cDest.Bearing);
    cDest.Bearing *= CRadians::PI.GetValue()/180.0;
    // Add it to the list of destinations
    m_vecDestinations.push_back(cDest);
  }
}

/****************************************/
/****************************************/

void CSimplifiedPathPlanningRVRLoopFunctions::ParseRandomDistributionParameters(TConfigurationNode&  tParametersNode) {
  /* Example of parameters required for positioning robots randomly
  <pathplanning randomDistribution="true" sharedDestination="true/false" robots="aNumber">
    <distribute>
      <position method="uniform" min="-2,-2,0" max="-1,2,0" />
      <orientation method="uniform" min="0,0,0" max="360,0,0" />
      <entity max_trials="100" />
    </distribute>
  </pathplanning> */
  TConfigurationNode& cPositionGenerator = GetNode(tParametersNode, "distribute");
  /* Get the needed nodes */
  TConfigurationNode cPositionNode;
  cPositionNode = GetNode(cPositionGenerator, "position");
  TConfigurationNode cOrientationNode;
  cOrientationNode = GetNode(cPositionGenerator, "orientation");
  TConfigurationNode cEntityNode;
  cEntityNode = GetNode(cPositionGenerator, "entity");
  /* Create the real number generators */
  m_pcPositionGenerator = CreateGenerator(cPositionNode);
  m_pcOrientationGenerator = CreateGenerator(cOrientationNode);
  /* How many trials before failing? */
  GetNodeAttribute(cEntityNode, "max_trials", m_unMaxTrials);

  m_unSeed = GetSimulator().GetRandomSeed();
}

/****************************************/
/****************************************/

void CSimplifiedPathPlanningRVRLoopFunctions::PreStep() {
  if (m_unStepCounter > m_unSetupTime) {
    m_bRobotsPerceived = true;
  }
  if (m_unStepCounter%50 == 0) { //try new assignment every 5seconds
    m_waypoints.clear();
  }

	if (m_mapRobots.empty()){
		if (m_bRandomDistribution)
			GenerateRandomDestinations();
		CSpace::TMapPerType& m_cRobots = GetSpace().GetEntitiesByType(m_strEntityType);
		UInt16 unCount = 0;
		for(CSpace::TMapPerType::iterator it = m_cRobots.begin(); it != m_cRobots.end(); ++it) {
			try {
				CVoidController* cController;
				if (m_strEntityType == "rvr"){
					CRVREntity& cRVR = *any_cast<CRVREntity*>(it->second);
					cController = &(dynamic_cast<CVoidController&>(cRVR.GetControllableEntity().GetController()));
				}
				else
					THROW_ARGOSEXCEPTION("Incorrect robot")
				m_mapRobots.insert(std::pair<UInt16,CVoidController*>(unCount,cController));
				++unCount;
			}
			catch(CARGoSException& ex) {
				LOGERR << ex.what() << "Verify that the controller for the rvr is void !\nThe system only support epuck, footbot & rvr\n";
				THROW_ARGOSEXCEPTION("Error on initializing list of the robots.")
			}
		}
	}
	// If no path for any robot (no destination), we should assign and generate them
	if (m_waypoints.empty() && m_bRobotsPerceived) {
		std::vector<CVirtualSensorDataPosition2D> vecPositions;
		for(UInt16 i = 0; i < m_mapRobots.size(); ++i) {
			vecPositions.push_back(m_mapRobots[i]->GetGPSSensor()->GetReading());
      //DEBUG:
      //LOG << m_mapRobots[i]->GetGPSSensor()->GetReading().XRange << "," << m_mapRobots[i]->GetGPSSensor()->GetReading().YRange << std::endl;
		}
		// Make assignment
		m_vecAssignment = m_cAssignmentAlgorithm->MakeDecisionAssignmentPath(vecPositions, m_vecDestinations);
		/* m_vecAssignment[i] now contains the number of the destination for robot i
		 * For each assigment, generate the path
		 */
		for(UInt16 i = 0; i < m_mapRobots.size(); ++i) {
    		//Put the waypoints to the destination into waypoints*/
			m_waypoints.push_back(m_cPathAlgorithm->GeneratePathFromTo(vecPositions[i],m_vecDestinations[m_vecAssignment[i]]));
			//Put first waypoint as destination of the robot */
			if (m_waypoints[i].size() > 0)
				(dynamic_cast<CDestinationVirtualSensor*> (m_mapRobots[i]->GetDestinationSensor()))->SetNextReading(m_waypoints[i][0]);
			else // IF unable to assign to the destination, to avoid seg fault, do not move
				m_waypoints[i].push_back((dynamic_cast<CDestinationVirtualSensor*> (m_mapRobots[i]->GetDestinationSensor()))->GetReading());
		}
  }
  m_unStepCounter++;
}

/****************************************/
/****************************************/

bool CSimplifiedPathPlanningRVRLoopFunctions::IsExperimentFinished() {
	return m_bAllRobotsInPosition;
}

/****************************************/
/****************************************/

void CSimplifiedPathPlanningRVRLoopFunctions::PostExperiment() {
	// write time of experiment or path planning to a file
	m_loopTime_list_file << "\t" << GetSpace().GetSimulationClock() << std::endl;
}

/****************************************/
/****************************************/

void CSimplifiedPathPlanningRVRLoopFunctions::GenerateRandomDestinations(){
	LOG << "USING SEED : "<< m_unSeed <<"\n";
	CSimulator::GetInstance().GetRNG()->SetSeed(m_unSeed);
	CSimulator::GetInstance().GetRNG()->Reset();
	m_vecDestinations.clear();

//		m_vecDestinations.push_back(std::vector<CVirtualSensorDataPosition2D>());
	UInt16 unTry = 0;
	for (UInt32 j=0; j<m_unTotalNumberRobot; ++j) {
		/* Set the position */
		CVector3 cPos = (*m_pcPositionGenerator)(unTry>0);
		/* Set the orientation */
		CVector3 cOri = (*m_pcOrientationGenerator)(unTry>0);

		CVirtualSensorDataPosition2D cDest;
		cDest.XRange = cPos.GetX();
		cDest.YRange = cPos.GetY();
		cDest.Bearing = cOri.GetZ()/180.0*CRadians::PI.GetValue();
		SBoundingBox myBoundingBox;
		myBoundingBox.MinCorner = CVector3(cDest.XRange-m_fRobotBodyRadius-0.03, cDest.YRange-m_fRobotBodyRadius-0.03, 0);
		myBoundingBox.MaxCorner = CVector3(cDest.XRange+m_fRobotBodyRadius+0.03, cDest.YRange+m_fRobotBodyRadius+0.03, 1);
		bool bCollide = false;

		/*
		 * Test collisions
		 */
		// With another destination
		for (UInt32 k=0; k<j && !bCollide; ++k) {
			if (cDest.IsColliding(m_vecDestinations[k], m_fRobotBodyRadius+0.03)) {
				bCollide = true;
				//DEBUG("Collide with other destination\n")
			}
		}
		// With boxes
		if (!bCollide){
			try {
				CSpace::TMapPerType& m_cBox = GetSpace().GetEntitiesByType("box");

				for(CSpace::TMapPerType::iterator it = m_cBox.begin(); it != m_cBox.end() && !bCollide; ++it) {
					CBoxEntity& cBox = *any_cast<CBoxEntity*>(it->second);
					if (cBox.GetEmbodiedEntity().GetBoundingBox().Intersects(myBoundingBox)) {
						bCollide = true;
						//DEBUG("Collide with box\n")
					}
				}
			}
			catch(CARGoSException& e){
			}
		}
		// With cylinders
		if (!bCollide){
			try{
				CSpace::TMapPerType& m_cCylinder = GetSpace().GetEntitiesByType("cylinder");
				for(CSpace::TMapPerType::iterator it = m_cCylinder.begin(); it != m_cCylinder.end() && !bCollide; ++it) {
					CCylinderEntity& cCylinder = *any_cast<CCylinderEntity*>(it->second);
					if (cCylinder.GetEmbodiedEntity().GetBoundingBox().Intersects(myBoundingBox)) {
						bCollide = true;
						//DEBUG("Collide with cylinder\n")
					}
				}
			}
			catch(CARGoSException& e){
			}
		}

		// If no collision, correct position
		if (!bCollide){
			m_vecDestinations.push_back(cDest);
			//DEBUG("POSITION GENERATED : %f %f %f\n",cDest.XRange, cDest.YRange, cDest.Bearing);
			unTry=0;
		}
		else{
			++unTry;
			--j;
			if (unTry > m_unMaxTrials)
				THROW_ARGOSEXCEPTION("UNABLE TO GENERATE DESTINATIONS NOT COLLIDING");
		}
	}
}

/****************************************/
/****************************************/

void CSimplifiedPathPlanningRVRLoopFunctions::Reset() {
	LOGERR << "PATH PLANNING NOT FINISHED\n";
	m_waypoints.clear();
	m_mapRobots.clear();
	m_unInPosition=0;
  m_bRobotsPerceived=false;
  m_unStepCounter=0;
}

/****************************************/
/****************************************/

void CSimplifiedPathPlanningRVRLoopFunctions::Destroy() {
	//delete m_pcFloor;
	delete m_pcPositionGenerator;
	delete m_pcOrientationGenerator;
}

/****************************************/
/****************************************/

CColor CSimplifiedPathPlanningRVRLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
  return CColor::WHITE;
}

/****************************************/
/****************************************/

void CSimplifiedPathPlanningRVRLoopFunctions::PostStep() {
	// Verify that the robots are in position
  if (m_bRobotsPerceived) {
  	UInt32 unNumberRobotPositioned =0;
  	for (UInt16 robot=0; robot < m_unTotalNumberRobot; robot++) {
  		CVirtualSensorDataPosition2D cPos = m_mapRobots[robot]->GetGPSSensor()->GetReading();
  		CVirtualSensorDataPosition2D cDest = m_mapRobots[robot]->GetDestinationSensor()->GetReading();
  		// if under 5 cm, consider as in position
		//LOG << CPathGenerator::LengthBetween(cPos,cDest) << std::endl;
  		if (CPathGenerator::LengthBetween(cPos,cDest) < 0.05) {
  			if (m_waypoints[robot].size()==1)
  				++unNumberRobotPositioned;
  			else{
  				// if the destination was not final, update it
  				m_waypoints[robot].erase(m_waypoints[robot].begin());
  	    	(dynamic_cast<CDestinationVirtualSensor*> (m_mapRobots[robot]->GetDestinationSensor()))->SetNextReading(m_waypoints[robot][0]);
  			}
  		}
  		// Verify next robot (one per timestep) if we can still reach his destination
  		else if ( (GetSpace().GetSimulationClock() + robot) % m_unTotalNumberRobot == 0 ) {
  			bool bIntersection = true;
  			if (m_waypoints[robot].size() > 1){
  				bIntersection = false;
  				// if next destination reachable -> remove current destination from list
  				CRay3 cRay = CRay3(CVector3(cPos.XRange,cPos.YRange,0.01),CVector3(m_waypoints[robot][1].XRange,m_waypoints[robot][1].YRange,0.1));
  				std::set<CEmbodiedEntity*> setIntersections = CPathGenerator::GetAllEmbodiedEntityIntersectedByRay(cRay);
  				for (std::set<CEmbodiedEntity*>::iterator it=setIntersections.begin(); it!=setIntersections.end() && !bIntersection; ++it){
  					if ((*it)->GetParent().GetTypeDescription() == "box" || (*it)->GetParent().GetTypeDescription() == "cylinder"){
  						bIntersection = true;
  					}
  				}
  				if (!bIntersection){
  					m_waypoints[robot].erase(m_waypoints[robot].begin());
  		    		(dynamic_cast<CDestinationVirtualSensor*> (m_mapRobots[robot]->GetDestinationSensor()))->SetNextReading(m_waypoints[robot][0]);
  				}
  			}
  			if (bIntersection) {
  				bIntersection = false;
  				// if destination is not reachable, add destination that allows to reach current destination
  				CRay3 cRay = CRay3(CVector3(cPos.XRange,cPos.YRange,0.01),CVector3(m_waypoints[robot][0].XRange,m_waypoints[robot][0].YRange,0.1));
  				std::set<CEmbodiedEntity*> setIntersections = CPathGenerator::GetAllEmbodiedEntityIntersectedByRay(cRay);
  				for (std::set<CEmbodiedEntity*>::iterator it=setIntersections.begin(); it!=setIntersections.end() && !bIntersection; ++it){
  					if ((*it)->GetParent().GetTypeDescription() == "box" || (*it)->GetParent().GetTypeDescription() == "cylinder"){
  						bIntersection = true;
  					}
  				}
  				if (bIntersection){
  					// There is a intersection between this robot and current destination
  					// -> Re-pathplanne
  					m_waypoints[robot] = m_cPathAlgorithm->GeneratePathFromTo(m_mapRobots[robot]->GetGPSSensor()->GetReading(),m_vecDestinations[m_vecAssignment[robot]]);
  		    		(dynamic_cast<CDestinationVirtualSensor*> (m_mapRobots[robot]->GetDestinationSensor()))->SetNextReading(m_waypoints[robot][0]);
  				}
  			}


  		}
  	}
  	/* if at least PlacementBudget robots */
  	if (unNumberRobotPositioned >= m_fBudgetPlacement * m_unTotalNumberRobot){
  		if (m_unInPosition == m_unTimeStepInPosition){
  			m_waypoints.clear();
  			//++unCurSequence;
        m_bAllRobotsInPosition = true;
        m_unInPosition = 0;
  		}
  		if (unNumberRobotPositioned == m_unTotalNumberRobot)
  			++m_unInPosition;
  		else
  			++m_unBudgetPlacementTimer;
  		if ( m_unMaxBudgetPlacementTimer && m_unBudgetPlacementTimer >= m_unMaxBudgetPlacementTimer){
  			m_waypoints.clear();
  			//++unCurSequence;
        m_bAllRobotsInPosition = true;
  			m_unInPosition = 0;
  		}
  	}
  	else {
  		m_unInPosition=0;
  		m_unBudgetPlacementTimer=0;
  	}
  	/* Took too much time to place => Stop everything */
  	if (m_tickBudget && GetSpace().GetSimulationClock() >= m_tickBudget){
  		m_waypoints.clear();
  		//++unCurSequence;
      m_bAllRobotsInPosition = true;
  		m_unInPosition = 0;
  	}
  }
}

/****************************************/
/****************************************/

std::vector<CVirtualSensorDataPosition2D> CSimplifiedPathPlanningRVRLoopFunctions::GetPathForController(const CVoidController& c_controller){
	if (m_mapRobots.size() !=0) {
		for (UInt16 robot=0; robot < m_mapRobots.size(); ++robot){
			if (m_mapRobots[robot] == &c_controller)
				return GetPaths()[robot];
		}
		THROW_ARGOSEXCEPTION("DID NOT FIND THE CORRESPONDING CONTROLLER");
	}
	return std::vector<CVirtualSensorDataPosition2D>();
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CSimplifiedPathPlanningRVRLoopFunctions, "simplified_pathplanning_loop_functions")
