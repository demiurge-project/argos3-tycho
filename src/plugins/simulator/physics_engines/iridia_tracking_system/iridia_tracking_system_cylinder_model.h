/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/iridia_tracking_system_cylinder_model.h>
 *
 *
 * @author Bernard Mayeur
 */

#ifndef IRIDIA_TRACKING_SYSTEM_CYLINDER_MODEL_H
#define IRIDIA_TRACKING_SYSTEM_CYLINDER_MODEL_H

#include "iridia_tracking_system_circular_model.h"
#include <argos3/plugins/simulator/entities/cylinder_entity.h>

namespace argos {

   class CIridiaTrackingSystemCylinderModel : public CIridiaTrackingSystemCircularModel {

   public:

      CIridiaTrackingSystemCylinderModel(CIridiaTrackingSystem& c_engine,
	                                CCylinderEntity& c_entity);
      virtual ~CIridiaTrackingSystemCylinderModel();

      virtual void CalculateBoundingBox();

      virtual void UpdateEntityStatus();
      virtual bool IsCollidingWithSomething();

      virtual void MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation) {};

      virtual Real GetEntityRadius() const{
   	   return m_cCylinderEntity.GetRadius();
      }
   private:

      CCylinderEntity& m_cCylinderEntity;
   };

}

#endif
