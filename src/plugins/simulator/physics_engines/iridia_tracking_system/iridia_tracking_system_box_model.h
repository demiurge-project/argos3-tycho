/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/iridia_tracking_system_box_model.h>
 *
 *
 * @author Bernard Mayeur
 */

#ifndef IRIDIA_TRACKING_SYSTEM_BOX_MODEL_H
#define IRIDIA_TRACKING_SYSTEM_BOX_MODEL_H

#include "iridia_tracking_system_rectangular_model.h"
#include <argos3/plugins/simulator/entities/box_entity.h>

namespace argos {

   class CIridiaTrackingSystemBoxModel : public CIridiaTrackingSystemRectangularModel {

   public:

      CIridiaTrackingSystemBoxModel(CIridiaTrackingSystem& c_engine,
	                                CBoxEntity& c_entity);
      virtual ~CIridiaTrackingSystemBoxModel();

      virtual void CalculateBoundingBox();

      virtual void UpdateEntityStatus();
      virtual bool IsCollidingWithSomething();

      virtual void MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation) {};

      virtual Real GetHalfEntityXSize() const{
    	  return m_cBoxEntity.GetSize().GetX()/2;
      }
      virtual Real GetHalfEntityYSize() const{
    	  return m_cBoxEntity.GetSize().GetY()/2;
      }

   private:

      CBoxEntity& m_cBoxEntity;
      // Position of the 4 corners of the box
      // Defined counter-clockwise from Y axis
      //  1-4
      //  | |
      //  2-3
      CVector2 cVertices[4];
   };

}

#endif
