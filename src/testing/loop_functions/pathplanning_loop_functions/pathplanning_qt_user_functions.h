#ifndef PATHPLANNING_QT_USER_FUNCTIONS_H
#define PATHPLANNING_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "simplified_pathplanning_loop_functions.h"
#include <testing/controllers/path_planning_controller.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

class CPathPlanningQTUserFunctions : public CQTOpenGLUserFunctions {

public:
   CPathPlanningQTUserFunctions();
   virtual ~CPathPlanningQTUserFunctions() {}
   virtual void DrawInWorld();

protected:
   void DrawWaypoints(const std::vector<CVector3>& c_waypoints, const CColor& c_segment_color = CColor::RED);
   void Draw(CEPuckEntity& c_entity);

   CSimplifiedPathPlanningLoopFunctions& m_cPathPlanningLoopFunction;

   /* Font used for drawing robot's ID */
   QFont m_pcDrawIDFont;
};

#endif
