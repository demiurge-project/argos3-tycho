/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/iridia_tracking_system_footbot_model.h>
 *
 *
 * @author Mattia Salvaro
 */

#ifndef IRIDIA_TRACKING_SYSTEM_FOOTBOT_MODEL_H
#define IRIDIA_TRACKING_SYSTEM_FOOTBOT_MODEL_H

#include "iridia_tracking_system_circular_model.h"
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

namespace argos {

   class CIridiaTrackingSystemFootBotModel : public CIridiaTrackingSystemCircularModel {

   public:

      CIridiaTrackingSystemFootBotModel(CIridiaTrackingSystem& c_engine,
	                                CFootBotEntity& c_entity);
      virtual ~CIridiaTrackingSystemFootBotModel();

      virtual void CalculateBoundingBox();

      virtual void UpdateEntityStatus();

      void GetVitualSensorList(CCI_Sensor::TMap &map_simulated_sensors);

      virtual void MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation) {};

      virtual Real GetEntityRadius() const{
    	  return 0.085036758f;
      }
   private:

      CFootBotEntity& m_cFootBotEntity;
   };

}

#endif
