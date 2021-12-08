/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/iridia_tracking_system_footbot_model.cpp>
 *
 *
 * @author Mattia Salvaro
 */

#include "iridia_tracking_system_footbot_model.h"

namespace argos {

   CIridiaTrackingSystemFootBotModel::CIridiaTrackingSystemFootBotModel(CIridiaTrackingSystem& c_engine,
	                                                                CFootBotEntity& c_entity) :
		CIridiaTrackingSystemCircularModel(c_engine, c_entity.GetEmbodiedEntity()),
      m_cFootBotEntity(c_entity) {
           //DEBUG_FUNCTION_ENTER;
      fprintf(stderr, "[DEBUG] Creating foot-bot '%s' to engine '%s'\n", c_entity.GetId().c_str(), c_engine.GetId().c_str());
      //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   CIridiaTrackingSystemFootBotModel::~CIridiaTrackingSystemFootBotModel() {
       //DEBUG_FUNCTION_ENTER;
	fprintf(stderr, "[DEBUG] Destroying foot-bot '%s' in engine '%s'\n", m_cFootBotEntity.GetId().c_str(), m_cITS.GetId().c_str());
    //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   void CIridiaTrackingSystemFootBotModel::CalculateBoundingBox() {
       //DEBUG_FUNCTION_ENTER;
	fprintf(stderr, "[DEBUG] Calculating BB for foot-bot '%s' in engine '%s'\n", m_cFootBotEntity.GetId().c_str(), m_cITS.GetId().c_str());
	// m_sBoundingBox.MinCorner.Set(...);
	// m_sBoundingBox.MaxCorner.Set(...);
    //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   void CIridiaTrackingSystemFootBotModel::UpdateEntityStatus() {
       //DEBUG_FUNCTION_ENTER;
       m_cITS.PositionAndOrientationPhysicsToSpace(m_cArenaPosition, m_cArenaOrientation, m_cFootBotEntity.GetId());
       //m_cEmbodiedEntity.SetPosition(m_cArenaPosition);
       //m_cEmbodiedEntity.SetOrientation(m_cArenaOrientation);
       GetEmbodiedEntity().GetOriginAnchor().Position = m_cArenaPosition;
       GetEmbodiedEntity().GetOriginAnchor().Orientation = m_cArenaOrientation;
       //DEBUG_FUNCTION_EXIT;
   }

   /****************************************/
   /****************************************/

   void CIridiaTrackingSystemFootBotModel::GetVitualSensorList(CCI_Sensor::TMap &map_simulated_sensors) {

       //DEBUG_FUNCTION_ENTER;

       CCI_Sensor::TMap mapAllSensors = m_cFootBotEntity.GetControllableEntity().GetController().GetAllSensors();
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

   REGISTER_STANDARD_IRIDIA_TRACKING_SYSTEM_OPERATIONS_ON_ENTITY(CFootBotEntity, CIridiaTrackingSystemFootBotModel);

}
