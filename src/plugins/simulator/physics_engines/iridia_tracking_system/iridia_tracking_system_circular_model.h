/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/iridia_tracking_system_circular_model.h>
 *
 *
 * @author Bernard Mayeur
 */

#ifndef IRIDIA_TRACKING_SYSTEM_CIRCULAR_MODEL_H
#define IRIDIA_TRACKING_SYSTEM_CIRCULAR_MODEL_H

namespace argos {
   class CIridiaTrackingSystem;
}

#include "iridia_tracking_system_model.h"

#include <argos3/core/simulator/physics_engine/physics_model.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>


namespace argos {

   class CIridiaTrackingSystemCircularModel : public CIridiaTrackingSystemModel {

   public:

      CIridiaTrackingSystemCircularModel(CIridiaTrackingSystem& c_engine,
                                 CEmbodiedEntity& c_entity) :
                                	 CIridiaTrackingSystemModel(c_engine, c_entity){}

      virtual ~CIridiaTrackingSystemCircularModel() {}

      virtual Real GetEntityRadius() const = 0;

      virtual CEmbodiedEntity* CheckIntersectionWithRay(Real& f_t_on_ray, const CRay3& c_ray) const {
    		CRay2 cRay (CVector2(c_ray.GetStart().GetX(),c_ray.GetStart().GetY()),CVector2(c_ray.GetEnd().GetX(),c_ray.GetEnd().GetY()));
    		// Translation to have a relative position to the center of the BOX
    		cRay.SetStart(cRay.GetStart()-CVector2(m_cArenaPosition.GetX(),m_cArenaPosition.GetY()));
    		cRay.SetEnd(cRay.GetEnd()-CVector2(m_cArenaPosition.GetX(),m_cArenaPosition.GetY()));
    		// Check for intersection
    		if (CVector2(cRay.GetStart()) == cRay.GetEnd()){
    		   if (cRay.GetStart().Length()<=GetEntityRadius()){
    			   f_t_on_ray=1.0f;
    			   //return &m_cEmbodiedEntity;
             return (CEmbodiedEntity*)&GetEmbodiedEntity();
    		   }
    		   else
    			   return NULL;
		    }
    		/* Rotate the system to put the line horizontaly
    		 * Actually we got the circle at the center. Rotate it does not change anything.
    		 * The angle between horizontal and the line is = atan(fAngularCoef)
    		 * The system should be rotated of minus this angle
    		 */
    		Real fAngularCoef = (cRay.GetEnd().GetY() - cRay.GetStart().GetY()) / (cRay.GetEnd().GetX() - cRay.GetStart().GetX());
    		CRadians fAngleRotation = CRadians(atan(fAngularCoef));
    		cRay.SetStart(CVector2(cRay.GetStart()).Rotate(-fAngleRotation));
    		cRay.SetEnd(CVector2(cRay.GetEnd()).Rotate(-fAngleRotation));

    		// Intersection with the cylinder?
    		// ray line equation is Y = startY
    		// We want the length between this line and the center of the circle
    		// Distance = |startY|   (simplification due to x=0, y=0, rotation to have horizontal line)
    		Real fDistance = Abs<Real>(cRay.GetStart().GetY());
    		//DEBUG("DISTANCE BETWEEN LINE AND CENTER : %f\n",fDistance)
    		if (fDistance <= GetEntityRadius()){
    			/*
    			 * Intersection of the circle and the line:
    			 * resolve
    			 * Y = starty
    			 * X²+Y²=GetEntityRadius()²
    			 *
    			 * <=>
    			 * Y = starty
    			 * X²+ ( starty )² = GetEntityRadius()²
    			 *
    			 * <=>
    			 * Y = starty
    			 * X² =  GetEntityRadius()² - starty²
    			 *
    			 * 		x1 = + sqrt ( GetEntityRadius()² - starty² )
    			 * 		x2 = - sqrt ( GetEntityRadius()² - starty² )
    			 */
    			Real X1 = sqrt(GetEntityRadius()*GetEntityRadius() - cRay.GetStart().GetY()*cRay.GetStart().GetY());
    			Real X2 = -X1;

    			// if X1 < all points of the ray => the ray finish before to have an intersection
    			if (X1 < Min<Real>(cRay.GetStart().GetX(),cRay.GetEnd().GetX()))
    				return NULL;
    			// same for X2 > all points
    			if (X2 > Max<Real>(cRay.GetStart().GetX(),cRay.GetEnd().GetX()))
    				return NULL;
    			// if the ray start inside of the figure, the intersection is the start point
    			if (cRay.GetStart().GetX() > X2 && cRay.GetStart().GetX() < X1){
    				f_t_on_ray=0;
    			}
    			// General case : first intersection found is the one closer to the start point
    			else if (cRay.GetStart().GetX() > 0)
    				f_t_on_ray = (cRay.GetStart().GetX() - X1) / (cRay.GetStart().GetX() - cRay.GetEnd().GetX());
    			else
    				f_t_on_ray = (cRay.GetStart().GetX() - X2) / (cRay.GetStart().GetX() - cRay.GetEnd().GetX());
    			//return &m_cEmbodiedEntity;
          return (CEmbodiedEntity*)&GetEmbodiedEntity();
    		}
    		return NULL;
        }
    };
}

#endif
