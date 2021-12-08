/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/iridia_tracking_system_epuck_model.h>
 *
 *
 * @author Mattia Salvaro
 */

#ifndef IRIDIA_TRACKING_SYSTEM_EPUCK_MODEL_H
#define IRIDIA_TRACKING_SYSTEM_EPUCK_MODEL_H

#include "iridia_tracking_system_circular_model.h"

#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

namespace argos {

   class CIridiaTrackingSystemEPuckModel : public CIridiaTrackingSystemCircularModel {

   public:

      CIridiaTrackingSystemEPuckModel(CIridiaTrackingSystem& c_engine,
                                    CEPuckEntity& c_entity);

      virtual ~CIridiaTrackingSystemEPuckModel();

      virtual void CalculateBoundingBox();

      virtual void UpdateEntityStatus();

      virtual void MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation) {};

      void GetVitualSensorList(CCI_Sensor::TMap &map_simulated_sensors);

      virtual Real GetEntityRadius() const{
		  return 0.035f;
	  }

      virtual CEmbodiedEntity* CheckIntersectionWithRay(Real& f_t_on_ray, const CRay3& c_ray) const {
    	  return NULL;
      }
   private:

      CEPuckEntity& m_cEPuckEntity;
   };

}

#endif
