#ifndef PATH_PLANNING_LOOP_FUNCTIONS_H
#define PATH_PLANNING_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/generic/simulator/destination_virtual_sensor.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
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

class RealNumberGenerator;

using namespace argos;

class CPathPlanningLoopFunctions : public CLoopFunctions {
public:

   CPathPlanningLoopFunctions();
   virtual ~CPathPlanningLoopFunctions() {}

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
   void ExtractSeed();

   std::vector<std::vector<CVirtualSensorDataPosition2D> > m_destinations;
   std::map<UInt16,CVoidController*> m_mapRobots;
   std::vector<UInt32> m_vecAssignment;
   std::vector<std::vector<CVirtualSensorDataPosition2D> > m_waypoints;


   CAssignmentGenerator * m_cAssignmentAlgorithm;
   CPathGenerator * m_cPathAlgorithm;
   std::string strEntityType;
   Real fRobotBodyRadius;
   UInt32 unCurSequence, unSequences, unNumberRobot, unInPosition, unTimeStepInPosition;
   CFloorEntity* m_pcFloor;
   CFloorEntity* m_pcExpFloor;
   bool m_bPathPlanning;
   bool m_bForceStopReset;
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
   UInt32 current_run;
   std::string next_run;
};

#endif
