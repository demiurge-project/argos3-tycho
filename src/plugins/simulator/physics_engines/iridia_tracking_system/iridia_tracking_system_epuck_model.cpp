/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/iridia_tracking_system_epuck_model.cpp>
 *
 *
 * @author Mattia Salvaro
 */

#include "iridia_tracking_system_epuck_model.h"

namespace argos {

   CIridiaTrackingSystemEPuckModel::CIridiaTrackingSystemEPuckModel(CIridiaTrackingSystem& c_engine,
                                                                    CEPuckEntity& c_entity) :
		CIridiaTrackingSystemCircularModel(c_engine, c_entity.GetEmbodiedEntity()),
      m_cEPuckEntity(c_entity) {
           //DEBUG_FUNCTION_ENTER;
      fprintf(stderr, "[DEBUG] Creating e-puck '%s' to engine '%s'\n", c_entity.GetId().c_str(), c_engine.GetId().c_str());
          //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   CIridiaTrackingSystemEPuckModel::~CIridiaTrackingSystemEPuckModel() {
           //DEBUG_FUNCTION_ENTER;
    fprintf(stderr, "[DEBUG] Destroying e-puck '%s' in engine '%s'\n", m_cEPuckEntity.GetId().c_str(), m_cITS.GetId().c_str());
        //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   void CIridiaTrackingSystemEPuckModel::CalculateBoundingBox() {
       //DEBUG_FUNCTION_ENTER;

       GetBoundingBox().MinCorner.Set(m_cArenaPosition.GetX() - 0.04, m_cArenaPosition.GetY() - 0.04, 0);
       GetBoundingBox().MaxCorner.Set(m_cArenaPosition.GetX() + 0.04, m_cArenaPosition.GetY() + 0.04, 10);

       //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   void CIridiaTrackingSystemEPuckModel::UpdateEntityStatus() {
       //DEBUG_FUNCTION_ENTER;
       CalculateBoundingBox();
       // get the updated position and orientation from the tracking system
       m_cITS.PositionAndOrientationPhysicsToSpace(m_cArenaPosition, m_cArenaOrientation, m_cEPuckEntity.GetId());
       GetEmbodiedEntity().GetOriginAnchor().Orientation = m_cArenaOrientation;
       GetEmbodiedEntity().GetOriginAnchor().Position = m_cArenaPosition;
       //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   void CIridiaTrackingSystemEPuckModel::GetVitualSensorList(CCI_Sensor::TMap &map_simulated_sensors) {

       //DEBUG_FUNCTION_ENTER;

       CCI_Sensor::TMap mapAllSensors = m_cEPuckEntity.GetControllableEntity().GetController().GetAllSensors();
       CCI_Sensor::TMap::iterator itMapSensors;
       for (itMapSensors = mapAllSensors.begin(); itMapSensors != mapAllSensors.end(); itMapSensors++)
       {
           if ((*itMapSensors).first.compare("epuck_virtual_sensor") == 0) {
               map_simulated_sensors.insert(*itMapSensors);
           }
       }

       //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_IRIDIA_TRACKING_SYSTEM_OPERATIONS_ON_ENTITY(CEPuckEntity, CIridiaTrackingSystemEPuckModel);

}
