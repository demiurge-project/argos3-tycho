/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/iridia_tracking_system_box_model.cpp>
 *
 *
 * @author Bernard Mayeur
 */

#include "iridia_tracking_system_box_model.h"

namespace argos {

   CIridiaTrackingSystemBoxModel::CIridiaTrackingSystemBoxModel(CIridiaTrackingSystem& c_engine,
	                                                                CBoxEntity& c_entity) :
      CIridiaTrackingSystemRectangularModel(c_engine, c_entity.GetEmbodiedEntity()),
      m_cBoxEntity(c_entity) {
           //DEBUG_FUNCTION_ENTER;
      fprintf(stderr, "[DEBUG] Creating box '%s' to engine '%s'\n", c_entity.GetId().c_str(), c_engine.GetId().c_str());
      //DEBUG_FUNCTION_EXIT;

      CRadians cAngleToDrop,cAngleToRotate;
      m_cArenaOrientation.ToEulerAngles(cAngleToRotate,cAngleToDrop,cAngleToDrop);
      CVector2 cArenaPosition = CVector2(m_cArenaPosition.GetX(),m_cArenaPosition.GetY());
      cVertices[0] = CVector2(-m_cBoxEntity.GetSize().GetX()/2,  m_cBoxEntity.GetSize().GetY()/2).Rotate(cAngleToRotate)+cArenaPosition;
      cVertices[1] = CVector2(-m_cBoxEntity.GetSize().GetX()/2, -m_cBoxEntity.GetSize().GetY()/2).Rotate(cAngleToRotate)+cArenaPosition;
      cVertices[2] = CVector2( m_cBoxEntity.GetSize().GetX()/2, -m_cBoxEntity.GetSize().GetY()/2).Rotate(cAngleToRotate)+cArenaPosition;
      cVertices[3] = CVector2( m_cBoxEntity.GetSize().GetX()/2,  m_cBoxEntity.GetSize().GetY()/2).Rotate(cAngleToRotate)+cArenaPosition;
      GetBoundingBox().MinCorner.SetZ(GetEmbodiedEntity().GetOriginAnchor().Orientation.GetZ());
      GetBoundingBox().MaxCorner.SetZ(GetEmbodiedEntity().GetOriginAnchor().Position.GetZ() + m_cBoxEntity.GetSize().GetZ());
      CalculateBoundingBox();
   }

   /****************************************/
   /****************************************/

   CIridiaTrackingSystemBoxModel::~CIridiaTrackingSystemBoxModel() {
       //DEBUG_FUNCTION_ENTER;
	fprintf(stderr, "[DEBUG] Destroying box '%s' in engine '%s'\n", m_cBoxEntity.GetId().c_str(), m_cITS.GetId().c_str());
    //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   void CIridiaTrackingSystemBoxModel::CalculateBoundingBox() {
		GetBoundingBox().MinCorner.SetX(Min<Real>(Min<Real>(cVertices[0].GetX(),cVertices[1].GetX()),Min<Real>(cVertices[2].GetX(),cVertices[3].GetX())));
		GetBoundingBox().MinCorner.SetY(Min<Real>(Min<Real>(cVertices[0].GetY(),cVertices[1].GetY()),Min<Real>(cVertices[2].GetY(),cVertices[3].GetY())));
		GetBoundingBox().MaxCorner.SetX(Max<Real>(Max<Real>(cVertices[0].GetX(),cVertices[1].GetX()),Max<Real>(cVertices[2].GetX(),cVertices[3].GetX())));
		GetBoundingBox().MaxCorner.SetY(Max<Real>(Max<Real>(cVertices[0].GetY(),cVertices[1].GetY()),Max<Real>(cVertices[2].GetY(),cVertices[3].GetY())));
   }

   /****************************************/
   /****************************************/

   void CIridiaTrackingSystemBoxModel::UpdateEntityStatus() {
       //DEBUG_FUNCTION_ENTER;

       CalculateBoundingBox();

       m_cITS.PositionAndOrientationPhysicsToSpace(m_cArenaPosition, m_cArenaOrientation, m_cBoxEntity.GetId());
       GetEmbodiedEntity().GetOriginAnchor().Orientation = m_cArenaOrientation;
       GetEmbodiedEntity().GetOriginAnchor().Position = m_cArenaPosition;
    //    m_cEmbodiedEntity.SetPosition(m_cArenaPosition);
    //    m_cEmbodiedEntity.SetOrientation(m_cArenaOrientation);

       m_cBoxEntity.UpdateComponents();
       //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   bool CIridiaTrackingSystemBoxModel::IsCollidingWithSomething() {
	   //TODO: verify that my bounding box intersect with another bounding box.
	   // If so, potential colliding. Else
	   return false;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_IRIDIA_TRACKING_SYSTEM_OPERATIONS_ON_ENTITY(CBoxEntity, CIridiaTrackingSystemBoxModel);

}
