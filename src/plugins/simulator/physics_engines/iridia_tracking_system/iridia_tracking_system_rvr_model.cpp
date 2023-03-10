/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/iridia_tracking_system_rvr_model.cpp>
 *
 *
 * @author Mattia Salvaro
 */

#include "iridia_tracking_system_rvr_model.h"

namespace argos {

   CIridiaTrackingSystemRVRModel::CIridiaTrackingSystemRVRModel(CIridiaTrackingSystem& c_engine,
                                                                    CRVREntity& c_entity) :
		CIridiaTrackingSystemCircularModel(c_engine, c_entity.GetEmbodiedEntity()),
      m_cRVREntity(c_entity) {
           //DEBUG_FUNCTION_ENTER;
      fprintf(stderr, "[DEBUG] Creating rvr '%s' to engine '%s'\n", c_entity.GetId().c_str(), c_engine.GetId().c_str());
          //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   CIridiaTrackingSystemRVRModel::~CIridiaTrackingSystemRVRModel() {
           //DEBUG_FUNCTION_ENTER;
    fprintf(stderr, "[DEBUG] Destroying rvr '%s' in engine '%s'\n", m_cRVREntity.GetId().c_str(), m_cITS.GetId().c_str());
        //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   void CIridiaTrackingSystemRVRModel::CalculateBoundingBox() {
       //DEBUG_FUNCTION_ENTER;

       GetBoundingBox().MinCorner.Set(m_cArenaPosition.GetX() - 0.148, m_cArenaPosition.GetY() - 0.148, 0);
       GetBoundingBox().MaxCorner.Set(m_cArenaPosition.GetX() + 0.148, m_cArenaPosition.GetY() + 0.148, 10);

       //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   void CIridiaTrackingSystemRVRModel::UpdateEntityStatus() {
       //DEBUG_FUNCTION_ENTER;
       CalculateBoundingBox();
       // get the updated position and orientation from the tracking system
       m_cITS.PositionAndOrientationPhysicsToSpace(m_cArenaPosition, m_cArenaOrientation, m_cRVREntity.GetId());
       GetEmbodiedEntity().GetOriginAnchor().Orientation = m_cArenaOrientation;
       GetEmbodiedEntity().GetOriginAnchor().Position = m_cArenaPosition;
       //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   void CIridiaTrackingSystemRVRModel::GetVitualSensorList(CCI_Sensor::TMap &map_simulated_sensors) {

       //DEBUG_FUNCTION_ENTER;

       CCI_Sensor::TMap mapAllSensors = m_cRVREntity.GetControllableEntity().GetController().GetAllSensors();
       CCI_Sensor::TMap::iterator itMapSensors;
       for (itMapSensors = mapAllSensors.begin(); itMapSensors != mapAllSensors.end(); itMapSensors++)
       {
           if ((*itMapSensors).first.compare("rvr_virtual_sensor") == 0) {
               map_simulated_sensors.insert(*itMapSensors);
           }
       }

       //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_IRIDIA_TRACKING_SYSTEM_OPERATIONS_ON_ENTITY(CRVREntity, CIridiaTrackingSystemRVRModel);

}
