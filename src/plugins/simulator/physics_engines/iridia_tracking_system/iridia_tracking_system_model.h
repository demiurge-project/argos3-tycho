/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/iridia_tracking_system_model.h>
 *
 *
 * @author Mattia Salvaro
 */

#ifndef IRIDIA_TRACKING_SYSTEM_MODEL_H
#define IRIDIA_TRACKING_SYSTEM_MODEL_H

namespace argos {
   class CIridiaTrackingSystem;
}

#include "iridia_tracking_system.h"

#include <argos3/core/simulator/physics_engine/physics_model.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/simulator/space/positional_indices/space_hash.h>
#include <argos3/core/simulator/space/positional_indices/grid.h>


namespace argos {

   class CIridiaTrackingSystemModel : public CPhysicsModel {

   public:

      typedef std::map<std::string, CIridiaTrackingSystemModel*> TMap;

   public:

      CIridiaTrackingSystemModel(CIridiaTrackingSystem& c_engine,
                                 CEmbodiedEntity& c_entity) :
         CPhysicsModel(c_engine, c_entity),
         m_cITS(c_engine)
   	   {
    	  //m_cArenaPosition = GetEmbodiedEntity().GetPosition();
    	  //m_cArenaOrientation = GetEmbodiedEntity().GetOrientation();
          m_cArenaPosition = GetEmbodiedEntity().GetOriginAnchor().Position;
          m_cArenaOrientation = GetEmbodiedEntity().GetOriginAnchor().Orientation;
   	   }

      virtual ~CIridiaTrackingSystemModel() {}

      virtual bool MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation,
                          bool b_check_only = false) {
         return false;
      }

      virtual bool IsCollidingWithSomething() const {
        return false;
      }

      virtual void Reset() {}

      virtual void UpdateEntityStatus() = 0;
      virtual void UpdateFromEntityStatus() {}

      virtual CEmbodiedEntity* CheckIntersectionWithRay(Real& f_t_on_ray, const CRay3& c_ray) const = 0;

   protected:

      CIridiaTrackingSystem& m_cITS;
      CVector3 m_cArenaPosition;
      CQuaternion m_cArenaOrientation;

   };


	/****************************************/
	/****************************************/

	class CITSModelSpaceHashUpdater : public CSpaceHashUpdater<CIridiaTrackingSystemModel> {
	public:
		virtual void operator()(CAbstractSpaceHash<CIridiaTrackingSystemModel>& c_space_hash,
				CIridiaTrackingSystemModel& c_element){
			// Calculate the position of the LED in the space hash
			c_space_hash.SpaceToHashTable(m_nI, m_nJ, m_nK, c_element.GetEmbodiedEntity().GetOriginAnchor().Position);
			// Update the corresponding cell
			c_space_hash.UpdateCell(m_nI, m_nJ, m_nK, c_element);
		}
	private:
	  SInt32 m_nI, m_nJ, m_nK;
	};

	/****************************************/
	/****************************************/

	class CITSModelGridUpdater : public CGrid<CIridiaTrackingSystemModel>::COperation {
	public:
		CITSModelGridUpdater(CGrid<CIridiaTrackingSystemModel>& c_grid):
    		m_cGrid(c_grid), m_nI(0), m_nJ(0), m_nK(0) {}
	    virtual bool operator()(CIridiaTrackingSystemModel& c_model){
			try {
				// Calculate the position of the LED in the space hash
				m_cGrid.PositionToCell(m_nI, m_nJ, m_nK, c_model.GetEmbodiedEntity().GetOriginAnchor().Position);
				// Update the corresponding cell
				m_cGrid.UpdateCell(m_nI, m_nJ, m_nK, c_model);
			}
			catch(CARGoSException& ex) {
				THROW_ARGOSEXCEPTION("While updating the Model grid for the tracking system engine");
			}
			// Continue with the other entities
			return true;
		}
	private:
	    CGrid<CIridiaTrackingSystemModel>& m_cGrid;
	    SInt32 m_nI, m_nJ, m_nK;
    };
}

/****************************************/
/****************************************/

#endif
