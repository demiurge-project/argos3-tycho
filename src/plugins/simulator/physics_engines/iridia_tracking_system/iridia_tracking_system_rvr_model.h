/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/iridia_tracking_system_rvr_model.h>
 *
 *
 * @author Miquel Kegeleirs
 */

#ifndef IRIDIA_TRACKING_SYSTEM_RVR_MODEL_H
#define IRIDIA_TRACKING_SYSTEM_RVR_MODEL_H

#include "iridia_tracking_system_circular_model.h"

#include <argos3/plugins/robots/rvr/simulator/rvr_entity.h>

namespace argos {

   class CIridiaTrackingSystemRVRModel : public CIridiaTrackingSystemCircularModel {

   public:

      CIridiaTrackingSystemRVRModel(CIridiaTrackingSystem& c_engine,
                                    CRVREntity& c_entity);

      virtual ~CIridiaTrackingSystemRVRModel();

      virtual void CalculateBoundingBox();

      virtual void UpdateEntityStatus();

      virtual void MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation) {};

      void GetVitualSensorList(CCI_Sensor::TMap &map_simulated_sensors);

      virtual Real GetEntityRadius() const{
		  return 0.143f;
	  }

      virtual CEmbodiedEntity* CheckIntersectionWithRay(Real& f_t_on_ray, const CRay3& c_ray) const {
    	  return NULL;
      }
   private:

      CRVREntity& m_cRVREntity;
   };

}

#endif
