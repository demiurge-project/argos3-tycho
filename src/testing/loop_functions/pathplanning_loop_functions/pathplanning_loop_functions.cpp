#include "pathplanning_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <testing/controllers/path_planning_controller.h>
#include <stdio.h>
#include "position_generator.h"

/****************************************/
/****************************************/

CPathPlanningLoopFunctions::CPathPlanningLoopFunctions() :
    m_destinations(),
    m_cAssignmentAlgorithm(NULL),
    m_cPathAlgorithm(NULL),
    strEntityType(""),
    fRobotBodyRadius(0),
    unCurSequence(0),
    unSequences(0),
    unNumberRobot(0),
    unInPosition(0),
    unTimeStepInPosition(20),
    m_pcFloor(new CFloorEntity("loopFunctionFloor","src/testing/loop_functions/pathplanning_loop_functions/pathplanning.png")),
    m_pcExpFloor(NULL),
    m_bPathPlanning(true),
    m_bForceStopReset(true),
    m_bRandomDistribution(false),
    m_tickBudget(0),
    m_fBudgetPlacement(1),
    m_unBudgetPlacementTimer(0),
    m_unMaxBudgetPlacementTimer(0),
    m_pcExperimentLoopFunctions(NULL),
    m_pcPositionGenerator(NULL),
    m_pcOrientationGenerator(NULL),
    m_unMaxTrials(0),
    current_run(0){
}

/****************************************/
/****************************************/

void CPathPlanningLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
      TConfigurationNode& tPathPlanning = GetNode(t_node, "pathplanning");
      GetNodeAttribute(tPathPlanning, "randomDistribution", m_bRandomDistribution);
      GetNodeAttribute(tPathPlanning, "sequence", unSequences);
      GetNodeAttribute(tPathPlanning, "robots", unNumberRobot);
      GetNodeAttribute(tPathPlanning, "robotType", strEntityType);
      GetNodeAttributeOrDefault(tPathPlanning, "timestepInPosition", unTimeStepInPosition, unTimeStepInPosition);
      GetNodeAttributeOrDefault(tPathPlanning, "tickBudget", m_tickBudget, m_tickBudget);
      GetNodeAttributeOrDefault(tPathPlanning, "minProcentPlacementBudget", m_fBudgetPlacement, m_fBudgetPlacement);
      GetNodeAttributeOrDefault(tPathPlanning, "maxTimeAfterBudgetPlacement", m_unMaxBudgetPlacementTimer, m_unMaxBudgetPlacementTimer);

      /* INIT GRAPH AND ASSIGNMENT TYPE */
      std::string strParam = "min_max_assignment_generator";
      GetNodeAttributeOrDefault(tPathPlanning, "assigment", strParam, strParam);
      m_cAssignmentAlgorithm = CFactory<CAssignmentGenerator>::New(strParam);
      m_cAssignmentAlgorithm->Init(tPathPlanning);
      m_cPathAlgorithm = m_cAssignmentAlgorithm->GetPathGenerator();


      char destinationNumber[50];
      UInt8 size;
      if (strEntityType == "epuck"){
    	  fRobotBodyRadius = 0.035f;
	  }
	  else if (strEntityType == "foot-bot") {
		  fRobotBodyRadius = 0.085036758f;
	  }
	  else
		THROW_ARGOSEXCEPTION("Unkown robot type");

      /* Verify the number of robots defined in the argos configuration file */
      std::string strRobotType = (strEntityType == "epuck") ? "e-puck" : strEntityType;
      UInt8 unNumberEntityDefined = 0;
      TConfigurationNodeIterator itArenaItem;
	  for(itArenaItem = itArenaItem.begin(&GetNode(CSimulator::GetInstance().GetConfigurationRoot(), "arena"));
		  itArenaItem != itArenaItem.end();
		  ++itArenaItem) {
		  if(itArenaItem->Value() == strRobotType) {
			  unNumberEntityDefined++;
		  }
	  }
	  if (unNumberEntityDefined != unNumberRobot){
		  LOGERR <<"Incorrect number of robots defined, "<< unNumberEntityDefined <<" robots detected, "<< unNumberRobot<<" needed of type "<< strRobotType<< std::endl;
		  THROW_ARGOSEXCEPTION("Incorrect number of robots.");
	  }

      if (!m_bRandomDistribution) {
    	  // FIXED DISTRIBUTION of destination
    	  /*
    	    <pathplanning randomDistribution="false" sharedDestination="true/false" sequence="2" robots="3" robotType="epuck/foot-bot">
				<destinations1>
					<destination1 x="0.5" y="0.5" rot="-6.2" />
					<destination2 x="-0.5" y="0.5" rot="3.14" />
					<destination3 x="0" y="0.5" rot="1" />
				</destinations1>
				<destinations2>
					<destination1 x="0.75" y="0.75" rot="0" />
					<destination2 x="0" y="0.75" rot="0" />
					<destination3 x="-0.75" y="0.75" rot="0" />
				</destinations2>
			</pathplanning>

    	   */
    	  // For each sequence of destination
		  for (UInt32 i=0; i<unSequences; ++i) {
			  m_destinations.push_back(std::vector<CVirtualSensorDataPosition2D>());
			  size=sprintf (destinationNumber, "destinations%d", i+1);
			  if (size < 13)
				  THROW_ARGOSEXCEPTION("ERROR DURING PARSING OF DESTINATIONS NODES")
			  TConfigurationNode& tPathPlanningDestinationList = GetNode(tPathPlanning, destinationNumber);
			  // Read each destination
			  for (UInt32 j=0; j<unNumberRobot; ++j){
				  size=sprintf (destinationNumber, "destination%d", j+1);
				  if (size < 12)
					  THROW_ARGOSEXCEPTION("ERROR DURING PARSING OF DESTINATION NODES")
				  TConfigurationNode& tPathPlanningDestination = GetNode(tPathPlanningDestinationList, destinationNumber);
				  CVirtualSensorDataPosition2D cDest;
				  GetNodeAttribute(tPathPlanningDestination, "x", cDest.XRange);
				  GetNodeAttribute(tPathPlanningDestination, "y", cDest.YRange);
				  GetNodeAttribute(tPathPlanningDestination, "rot", cDest.Bearing);
				  cDest.Bearing *= CRadians::PI.GetValue()/180.0;
				  // Add it to the list of destinations
				  m_destinations.at(i).push_back(cDest);
			  }
		  }
      }
      else {
    	  //RANDOM DISTRIBUTION of destinations
    	  /*
    	    <pathplanning randomDistribution="true" sharedDestination="true/false" sequence="aNumber" robots="aNumber">
				<distribute>
				  <position method="uniform" min="-2,-2,0" max="-1,2,0" />
				  <orientation method="uniform" min="0,0,0" max="360,0,0" />
				  <entity max_trials="100" />
				</distribute>
			</pathplanning>
    	   */
    	  TConfigurationNode& cPositionGenerator = GetNode(tPathPlanning, "distribute");
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

		  try {
			  // Try to open file with seeds
			  std::string strSeeds;
			  GetNodeAttribute(cPositionGenerator, "seeds", strSeeds);
			  m_seeds_list_file.open(strSeeds.c_str(), std::ios::in);
			  if (!m_seeds_list_file) {
				  LOGERR << "Can't open input file " << strSeeds << std::endl;
				  THROW_ARGOSEXCEPTION("Can't open input file " + strSeeds);
			  }
			  // if found, extract first seed
			  ExtractSeed();
		  }
		  catch(CARGoSException& ex) {
			  LOGERR << "UNABLE TO FIND SEED FILE. ARGoS seed used\n";
			  current_run = GetSimulator().GetRandomSeed();
		  }
      }
      try {
    	  // Open file for time of experiments (for appending to it)
		  m_loopTime_list_file.open("loopTime.out", std::ios_base::app | std::ios_base::out);
		  if (!m_loopTime_list_file) {
			  LOGERR << "Can't open input file loopTime.out\n";
			  THROW_ARGOSEXCEPTION("Can't open input file  loopTime.out");
		  }
	  }
	  catch(CARGoSException& ex) {

	  }
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }
   m_pcExpFloor = &GetSpace().GetFloorEntity();
   GetSpace().SetFloorEntity(*m_pcFloor);
   try {
	   // Try to open user loop function
	   std::string strLibrary, strLabel;
	   GetNodeAttributeOrDefault(t_node, "experimentLibrary", strLibrary, strLibrary);
	   GetNodeAttributeOrDefault(t_node, "experimentLabel", strLabel, strLabel);
	   if(! strLibrary.empty()) {
		  // load it
		  CDynamicLoading::LoadLibrary(strLibrary);
		  if(! strLabel.empty()) {
			   m_pcExperimentLoopFunctions = CFactory<CLoopFunctions>::New(strLabel);
			   TConfigurationNode& cExperimentParam = GetNode(t_node, "experimentLoopFunctionParams");
			   m_pcExperimentLoopFunctions->Init(cExperimentParam);
		  }
	   }
	}
	catch(CARGoSException& ex) {
	   THROW_ARGOSEXCEPTION_NESTED("Error initializing experiment loop functions", ex);
	}
}

bool CPathPlanningLoopFunctions::IsExperimentFinished() {
	// If currently making experiment
	if (m_pcExperimentLoopFunctions && !m_bPathPlanning)
		return m_pcExperimentLoopFunctions->IsExperimentFinished();
	// else : making path planning
	// if last sequence : path planning is finished
	if (unCurSequence == unSequences)
		m_bForceStopReset = false;
	return unCurSequence == unSequences;
}

void CPathPlanningLoopFunctions::PostExperiment() {
	// write time of experiment or path planning to a file
	m_loopTime_list_file << "\t" << GetSpace().GetSimulationClock() << std::endl;
	// if just finish an experiment successfully
	if (    m_bRandomDistribution
		 && (     (unCurSequence == unSequences && !m_pcExperimentLoopFunctions)
			   || (   m_pcExperimentLoopFunctions
					  && !m_bPathPlanning
					  && (m_pcExperimentLoopFunctions->IsExperimentFinished() || GetSimulator().GetMaxSimulationClock() == GetSpace().GetSimulationClock())
				  )
			)
		){
		// extract the next seed
		ExtractSeed();
	}
	// if were making an experiment, call the corresponding function
	if (m_pcExperimentLoopFunctions && !m_bPathPlanning)
		m_pcExperimentLoopFunctions->PostExperiment();
}

void CPathPlanningLoopFunctions::ExtractSeed(){
	// If a file is open with seeds inside
	if (m_seeds_list_file){
		std::string run;
		do{
			run=next_run;
			current_run = atoi(run.c_str());
			if(!m_seeds_list_file.eof())
			{
				getline (m_seeds_list_file, next_run);

			}else{
				LOG<<"EXPERIMENT FINISHED!!!!!!\n";
				current_run = GetSimulator().GetRandomSeed();
				return;
			}
		}while(run.length()==0);
	}
	else {
		current_run = GetSimulator().GetRandomSeed();
	}
}

void CPathPlanningLoopFunctions::GenerateRandomDestinations(){
	LOG << "USING SEED : "<<current_run<<"\n";
	CSimulator::GetInstance().GetRNG()->SetSeed(current_run);
	CSimulator::GetInstance().GetRNG()->Reset();
	m_destinations.clear();
	for (UInt32 i=0; i<unSequences; ++i) {
		m_destinations.push_back(std::vector<CVirtualSensorDataPosition2D>());
		/* Same work than CSpace::Distribute() but cannot use it because of virtual sensors
		* We just need positions, not totally generated robot.
		*/
		UInt16 unTry = 0;
		for (UInt32 j=0; j<unNumberRobot; ++j) {
			/* Set the position */
			CVector3 cPos = (*m_pcPositionGenerator)(unTry>0);
			/* Set the orientation */
			CVector3 cOri = (*m_pcOrientationGenerator)(unTry>0);

			CVirtualSensorDataPosition2D cDest;
			cDest.XRange = cPos.GetX();
			cDest.YRange = cPos.GetY();
			cDest.Bearing = cOri.GetZ()/180.0*CRadians::PI.GetValue();
			SBoundingBox myBoundingBox;
			myBoundingBox.MinCorner = CVector3(cDest.XRange-fRobotBodyRadius-0.03, cDest.YRange-fRobotBodyRadius-0.03, 0);
			myBoundingBox.MaxCorner = CVector3(cDest.XRange+fRobotBodyRadius+0.03, cDest.YRange+fRobotBodyRadius+0.03, 1);
			bool bCollide = false;

			/*
			 * Test collisions
			 */
			// With another destination
			for (UInt32 k=0; k<j && !bCollide; ++k) {
				if (cDest.IsColliding(m_destinations[i][k], fRobotBodyRadius+0.03)) {
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
				m_destinations.at(i).push_back(cDest);
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
}


/****************************************/
/****************************************/

void CPathPlanningLoopFunctions::Reset() {
	// if we terminate experiment by pushing stop : robot should still pathplane
	if (m_bForceStopReset){
		LOGERR << "PATH PLANNING NOT FINISHED\n";
		m_waypoints.clear();
		unCurSequence=0;
		m_mapRobots.clear();
		unInPosition=0;
	}
	else {
		// if there is an experiment to execute, so we do
		if (m_bPathPlanning){
			// Change the floor entity to the one of the user
			if (m_pcExperimentLoopFunctions)
				GetSpace().SetFloorEntity(*m_pcExpFloor);
			m_waypoints.clear();
			unCurSequence=0;
			m_mapRobots.clear();
			unInPosition=0;
		}
		else{
			GetSpace().SetFloorEntity(*m_pcFloor);
			m_pcExperimentLoopFunctions->Reset();
			m_bForceStopReset = true;
		}
		if (m_pcExperimentLoopFunctions)
			m_bPathPlanning = !m_bPathPlanning;
	}
}

/****************************************/
/****************************************/

void CPathPlanningLoopFunctions::Destroy() {
	delete m_pcFloor;
	if (m_pcExperimentLoopFunctions)
		delete m_pcExperimentLoopFunctions;
	/* Delete the generators*/
	delete m_pcPositionGenerator;
	delete m_pcOrientationGenerator;
}

/****************************************/
/****************************************/

CColor CPathPlanningLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
	if (m_pcExperimentLoopFunctions && !m_bPathPlanning)
		return m_pcExperimentLoopFunctions->GetFloorColor(c_position_on_plane);
    return CColor::WHITE;
}

/****************************************/
/****************************************/

void CPathPlanningLoopFunctions::PreStep() {
	// If currently in experiment, call the corresponding function of the user
	if (m_pcExperimentLoopFunctions && !m_bPathPlanning)
		return m_pcExperimentLoopFunctions->PreStep();
	// If first experiment, map the robots to numbers
	if (m_mapRobots.empty()){
		if (m_bRandomDistribution)
			GenerateRandomDestinations();
		CSpace::TMapPerType& m_cRobots = GetSpace().GetEntitiesByType(strEntityType);
		UInt16 unCount = 0;
		for(CSpace::TMapPerType::iterator it = m_cRobots.begin(); it != m_cRobots.end(); ++it) {
			try {
				CVoidController* cController;
				if (strEntityType == "foot-bot"){
					CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
					cController = &(dynamic_cast<CVoidController&>(cFootBot.GetControllableEntity().GetController()));
				}
				else if (strEntityType == "epuck"){
					CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
					cController = &(dynamic_cast<CVoidController&>(cEPuck.GetControllableEntity().GetController()));
				}
				else
					THROW_ARGOSEXCEPTION("Incorrect robot")
				m_mapRobots.insert(std::pair<UInt16,CVoidController*>(unCount,cController));
				++unCount;
			}
			catch(CARGoSException& ex) {
				LOGERR << ex.what() << " Verify that the controller for the foot-bot or epuck is void !\nThe system only support epuck & footbot at time\n";
				THROW_ARGOSEXCEPTION("Error on initializing list of the robots.")
			}
		}
	}
	// If no path for any robot (no destination), we should assign and generate them
	if (m_waypoints.empty())
    {
		std::vector<CVirtualSensorDataPosition2D> vecPositions;
		for(UInt16 i = 0; i < m_mapRobots.size(); ++i) {
			vecPositions.push_back(m_mapRobots[i]->GetGPSSensor()->GetReading());
		}
		// Make assignment
		m_vecAssignment = m_cAssignmentAlgorithm->MakeDecisionAssignmentPath(vecPositions,m_destinations[unCurSequence]);
		/* m_vecAssignmentDestination[i] now contains the number of the destination for robot i
		 * For each assigment, generate the path
		 */
		for(UInt16 i = 0; i < m_mapRobots.size(); ++i) {
    		//Put the waypoints to the destination into waypoints*/
			m_waypoints.push_back(m_cPathAlgorithm->GeneratePathFromTo(vecPositions[i],m_destinations[unCurSequence][m_vecAssignment[i]]));
			//Put first waypoint as destination of the robot */
			if (m_waypoints[i].size() > 0)
				(dynamic_cast<CDestinationVirtualSensor*> (m_mapRobots[i]->GetDestinationSensor()))->SetNextReading(m_waypoints[i][0]);
			else // IF unable to assign to the destination, to avoid seg fault, do not move
				m_waypoints[i].push_back((dynamic_cast<CDestinationVirtualSensor*> (m_mapRobots[i]->GetDestinationSensor()))->GetReading());
		}
    }
}


void CPathPlanningLoopFunctions::PostStep() {
	// If currently in experiment, call the corresponding function of the user
	if (m_pcExperimentLoopFunctions && !m_bPathPlanning)
		return m_pcExperimentLoopFunctions->PostStep();
	// Verify that the robots are in position
	UInt32 unNumberRobotDestination =0;
	for (UInt16 robot=0; robot < unNumberRobot; robot++)
	{
		CVirtualSensorDataPosition2D cPos = m_mapRobots[robot]->GetGPSSensor()->GetReading();
		CVirtualSensorDataPosition2D cDest = m_mapRobots[robot]->GetDestinationSensor()->GetReading();
		// if under 5 cm, consider as in position
		if (CPathGenerator::LengthBetween(cPos,cDest) < 0.05)
		{
			if (m_waypoints[robot].size()==1)
				++ unNumberRobotDestination;
			else{
				// if the destination was not final, update it
				m_waypoints[robot].erase(m_waypoints[robot].begin());
	    		(dynamic_cast<CDestinationVirtualSensor*> (m_mapRobots[robot]->GetDestinationSensor()))->SetNextReading(m_waypoints[robot][0]);
			}
		}
		// Verify next robot (one per timestep) if we can still reach his destination
		else if ( (GetSpace().GetSimulationClock() + robot) % unNumberRobot == 0 ) {
			bool bIntersection = true;
			if (m_waypoints[robot].size() > 1){
				bIntersection = false;
				// if next destination reachable -> remove current destination from list
				CRay3 cRay = CRay3(CVector3(cPos.XRange,cPos.YRange,0.01),CVector3(m_waypoints[robot][1].XRange,m_waypoints[robot][1].YRange,0.01));
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
				CRay3 cRay = CRay3(CVector3(cPos.XRange,cPos.YRange,0.01),CVector3(m_waypoints[robot][0].XRange,m_waypoints[robot][0].YRange,0.01));
				std::set<CEmbodiedEntity*> setIntersections = CPathGenerator::GetAllEmbodiedEntityIntersectedByRay(cRay);
				for (std::set<CEmbodiedEntity*>::iterator it=setIntersections.begin(); it!=setIntersections.end() && !bIntersection; ++it){
					if ((*it)->GetParent().GetTypeDescription() == "box" || (*it)->GetParent().GetTypeDescription() == "cylinder"){
						bIntersection = true;
					}
				}
				if (bIntersection){
					// There is a intersection between this robot and current destination
					// -> Re-pathplanne
					m_waypoints[robot] = m_cPathAlgorithm->GeneratePathFromTo(m_mapRobots[robot]->GetGPSSensor()->GetReading(),m_destinations[unCurSequence][m_vecAssignment[robot]]);
		    		(dynamic_cast<CDestinationVirtualSensor*> (m_mapRobots[robot]->GetDestinationSensor()))->SetNextReading(m_waypoints[robot][0]);
				}
			}


		}
	}
	/* if at least PlacementBudget robots */
	if (unNumberRobotDestination >= m_fBudgetPlacement * unNumberRobot){
		if (unInPosition == unTimeStepInPosition){
			m_waypoints.clear();
			++unCurSequence;
			unInPosition = 0;
		}
		if (unNumberRobotDestination == unNumberRobot)
			++unInPosition;
		else
			++m_unBudgetPlacementTimer;
		if ( m_unMaxBudgetPlacementTimer && m_unBudgetPlacementTimer >= m_unMaxBudgetPlacementTimer){
			m_waypoints.clear();
			++unCurSequence;
			unInPosition = 0;
		}
	}
	else {
		unInPosition=0;
		m_unBudgetPlacementTimer=0;
	}
	/* Took too muck time to place => Stop everything */
	if (m_tickBudget && GetSpace().GetSimulationClock() >= m_tickBudget){
		m_waypoints.clear();
		++unCurSequence;
		unInPosition = 0;
	}
}

std::vector<CVirtualSensorDataPosition2D> CPathPlanningLoopFunctions::GetPathForController(const CVoidController& c_controller){
	if (m_mapRobots.size() !=0)
	{
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

REGISTER_LOOP_FUNCTIONS(CPathPlanningLoopFunctions, "pathplanning_loop_functions")
