/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/iridia_tracking_system_cylinder_model.cpp>
 *
 *
 * @author Bernard Mayeur
 */

#include "iridia_tracking_system_cylinder_model.h"

namespace argos {

   CIridiaTrackingSystemCylinderModel::CIridiaTrackingSystemCylinderModel(CIridiaTrackingSystem& c_engine,
	                                                                CCylinderEntity& c_entity) :
      CIridiaTrackingSystemCircularModel(c_engine, c_entity.GetEmbodiedEntity()),
      m_cCylinderEntity(c_entity) {
           //DEBUG_FUNCTION_ENTER;
      fprintf(stderr, "[DEBUG] Creating cylinder '%s' to engine '%s'\n", c_entity.GetId().c_str(), c_engine.GetId().c_str());
      //DEBUG_FUNCTION_EXIT;
      GetBoundingBox().MinCorner.SetZ(GetEmbodiedEntity().GetOriginAnchor().Position.GetZ());
      GetBoundingBox().MaxCorner.SetZ(GetEmbodiedEntity().GetOriginAnchor().Position.GetZ() + m_cCylinderEntity.GetHeight());
      CalculateBoundingBox();
   }

   /****************************************/
   /****************************************/

   CIridiaTrackingSystemCylinderModel::~CIridiaTrackingSystemCylinderModel() {
       //DEBUG_FUNCTION_ENTER;
	fprintf(stderr, "[DEBUG] Destroying cylinder '%s' in engine '%s'\n", m_cCylinderEntity.GetId().c_str(), m_cITS.GetId().c_str());
    //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   void CIridiaTrackingSystemCylinderModel::CalculateBoundingBox() {
		GetBoundingBox().MinCorner.SetX(m_cArenaPosition.GetX()-m_cCylinderEntity.GetRadius());
		GetBoundingBox().MinCorner.SetY(m_cArenaPosition.GetY()-m_cCylinderEntity.GetRadius());
		GetBoundingBox().MaxCorner.SetX(m_cArenaPosition.GetX()+m_cCylinderEntity.GetRadius());
		GetBoundingBox().MaxCorner.SetY(m_cArenaPosition.GetY()+m_cCylinderEntity.GetRadius());
   }

   /****************************************/
   /****************************************/

   void CIridiaTrackingSystemCylinderModel::UpdateEntityStatus() {
       //DEBUG_FUNCTION_ENTER;

       CalculateBoundingBox();

       m_cITS.PositionAndOrientationPhysicsToSpace(m_cArenaPosition, m_cArenaOrientation, m_cCylinderEntity.GetId());
       GetEmbodiedEntity().GetOriginAnchor().Orientation = m_cArenaOrientation;
       GetEmbodiedEntity().GetOriginAnchor().Position = m_cArenaPosition;
    //    m_cEmbodiedEntity.SetPosition(m_cArenaPosition);
    //    m_cEmbodiedEntity.SetOrientation(m_cArenaOrientation);

       m_cCylinderEntity.UpdateComponents();
       //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   bool CIridiaTrackingSystemCylinderModel::IsCollidingWithSomething() {
	   //TODO: verify that my bounding box intersect with another bounding box.
	   // If so, potential colliding. Else
	   return false;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_IRIDIA_TRACKING_SYSTEM_OPERATIONS_ON_ENTITY(CCylinderEntity, CIridiaTrackingSystemCylinderModel);

}
