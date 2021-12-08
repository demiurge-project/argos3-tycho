#include "pathplanning_qt_user_functions.h"

using namespace argos;

/****************************************/
/****************************************/

CPathPlanningQTUserFunctions::CPathPlanningQTUserFunctions() :
	m_cPathPlanningLoopFunction(dynamic_cast<CSimplifiedPathPlanningLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) {
	RegisterUserFunction<CPathPlanningQTUserFunctions,CEPuckEntity>(&CPathPlanningQTUserFunctions::Draw);
	m_pcDrawIDFont            = QFont("Times", 12, 0, false);
}

/****************************************/
/****************************************/

void CPathPlanningQTUserFunctions::DrawInWorld() {
   /* Go through all the robot paths and draw them */
   std::vector<std::vector<CVirtualSensorDataPosition2D> > vecWaypoints = m_cPathPlanningLoopFunction.GetPaths();
   for(UInt16 i = 0; i< vecWaypoints.size(); ++i){
	  std::vector<CVector3> cWaypoints;
	  CVoidController* pcController = m_cPathPlanningLoopFunction.getController(i);
	  cWaypoints.push_back(CVector3(pcController->GetGPSSensor()->GetReading().XRange,pcController->GetGPSSensor()->GetReading().YRange ,0.02));
	  cWaypoints.push_back(CVector3(pcController->GetDestinationSensor()->GetReading().XRange,pcController->GetDestinationSensor()->GetReading().YRange ,0.02));
      DrawWaypoints(cWaypoints);
      cWaypoints.clear();
      for (UInt16 j=0; j< vecWaypoints[i].size(); ++j){
    	  cWaypoints.push_back(CVector3(vecWaypoints[i][j].XRange, vecWaypoints[i][j].YRange, 0.01));
      }
      DrawWaypoints(cWaypoints, CColor::BLUE);
   }
}

/****************************************/
/****************************************/

void CPathPlanningQTUserFunctions::DrawWaypoints(const std::vector<CVector3>& c_waypoints, const CColor& c_segment_color) {
   /* Start drawing segments when you have at least two points */
   if(c_waypoints.size() > 1) {
      size_t unStart = 0;
      size_t unEnd = 1;
      while(unEnd < c_waypoints.size()) {
         DrawRay(CRay3(c_waypoints[unEnd], c_waypoints[unStart]), c_segment_color, 0.1f);
         ++unStart;
         ++unEnd;
      }
   }
}

/****************************************/
/****************************************/

void CPathPlanningQTUserFunctions::Draw(CEPuckEntity& c_entity) {
  DrawText(CVector3(-0.01, -0.01, 0.14), c_entity.GetId().substr(8,9).c_str(), CColor::BLUE, m_pcDrawIDFont);
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CPathPlanningQTUserFunctions, "pathplanning_qt_user_functions")
