#ifndef PATHPLANNING_RVR_QT_USER_FUNCTIONS_H
#define PATHPLANNING_RVR_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/rvr/simulator/rvr_entity.h>
#include "simplified_pathplanning_rvr_loop_functions.h"
#include <testing/controllers/path_planning_controller_rvr.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

class CPathPlanningRVRQTUserFunctions : public CQTOpenGLUserFunctions {

public:
   CPathPlanningRVRQTUserFunctions();
   virtual ~CPathPlanningRVRQTUserFunctions() {}
   virtual void DrawInWorld();

protected:
   void DrawWaypoints(const std::vector<CVector3>& c_waypoints, const CColor& c_segment_color = CColor::RED);
   void Draw(CRVREntity& c_entity);

   CSimplifiedPathPlanningRVRLoopFunctions& m_cPathPlanningRVRLoopFunction;

   /* Font used for drawing robot's ID */
   QFont m_pcDrawIDFont;
};

#endif
