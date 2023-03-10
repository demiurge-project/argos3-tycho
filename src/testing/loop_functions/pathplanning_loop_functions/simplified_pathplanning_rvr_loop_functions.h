#ifndef SIMPLIFIED_PATH_PLANNING_RVR_LOOP_FUNCTIONS_H
#define SIMPLIFIED_PATH_PLANNING_RVR_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/generic/simulator/destination_virtual_sensor.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/robots/rvr/simulator/rvr_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/utility/datatypes/any.h>
#include "../../controllers/void_controller.h"
#include "assignment_generator.h"
#include "path_generator.h"
#include <limits>
#include <set>
#include <argos3/core/utility/plugins/dynamic_loading.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <testing/controllers/path_planning_controller_rvr.h>
#include <stdio.h>

class RealNumberGenerator;

using namespace argos;

class CSimplifiedPathPlanningRVRLoopFunctions : public CLoopFunctions {
public:

   CSimplifiedPathPlanningRVRLoopFunctions();
   virtual ~CSimplifiedPathPlanningRVRLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();
   virtual void PostStep();
   virtual bool IsExperimentFinished();
   virtual void PostExperiment();

   virtual std::vector<CVirtualSensorDataPosition2D> GetPathForController(const CVoidController& c_controller);

   inline virtual std::vector<std::vector<CVirtualSensorDataPosition2D> > GetPaths(){
	   return m_waypoints;
   }

   inline virtual std::vector<CVector2> GetWaypointsPosition(){
   	   return m_cPathAlgorithm->GetWaypointsPosition();
   }

   inline virtual std::vector<std::vector<Real> > GetWaypointsMatrix(){
	   return m_cPathAlgorithm->GetWaypointsMatrix();
   }

   inline virtual CVoidController* getController(UInt16 i) {
	   return m_mapRobots[i];
   }

private:
   void GenerateRandomDestinations();
   void ParseFixedDestinations(TConfigurationNode& t_node);
   void ParseRandomDistributionParameters(TConfigurationNode& t_node);

   std::vector<CVirtualSensorDataPosition2D> m_vecDestinations;
   std::map<UInt16,CVoidController*> m_mapRobots;
   std::vector<UInt32> m_vecAssignment;
   std::vector<std::vector<CVirtualSensorDataPosition2D> > m_waypoints;

   CAssignmentGenerator * m_cAssignmentAlgorithm;
   CPathGenerator * m_cPathAlgorithm;
   std::string m_strEntityType;
   Real m_fRobotBodyRadius;
   UInt32 m_unSeed;
   UInt32 m_unTotalNumberRobot;
   UInt32 m_unInPosition;
   UInt32  m_unTimeStepInPosition;
   CFloorEntity* m_pcFloor;
   CFloorEntity* m_pcExpFloor;
   bool m_bPathPlanning;
   bool m_bForceStopReset;
   bool m_bAllRobotsInPosition;
   bool m_bRandomDistribution;
   UInt32 m_tickBudget;
   Real m_fBudgetPlacement;
   UInt32 m_unBudgetPlacementTimer;
   UInt32 m_unMaxBudgetPlacementTimer;
   CLoopFunctions* m_pcExperimentLoopFunctions;
   std::ifstream m_seeds_list_file;
   std::ofstream m_loopTime_list_file;
   RealNumberGenerator* m_pcPositionGenerator;
   RealNumberGenerator* m_pcOrientationGenerator;
   UInt32 m_unMaxTrials;

   /* Time needed for the tracking system to scan and recognize all tags in arena */
   UInt16 m_unSetupTime;

   UInt32 m_unStepCounter;

   /* Flag that tells whether the tracking system has had enough time to perceive all tags*/
   bool m_bRobotsPerceived;
};

#endif
