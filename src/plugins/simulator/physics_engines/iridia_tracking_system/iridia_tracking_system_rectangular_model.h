/**
 * @file <argos3/plugins/simulator/physics_engines/iridia_tracking_system/iridia_tracking_system_rectangular_model.h>
 *
 *
 * @author Bernard Mayeur
 */

#ifndef IRIDIA_TRACKING_SYSTEM_RECTANGULAR_MODEL_H
#define IRIDIA_TRACKING_SYSTEM_RECTANGULAR_MODEL_H

namespace argos {
   class CIridiaTrackingSystem;
}

#include "iridia_tracking_system_model.h"

#include <argos3/core/simulator/physics_engine/physics_model.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>


namespace argos {

   class CIridiaTrackingSystemRectangularModel : public CIridiaTrackingSystemModel {

   public:

	   CIridiaTrackingSystemRectangularModel(CIridiaTrackingSystem& c_engine,
                                 CEmbodiedEntity& c_entity) :
                                	 CIridiaTrackingSystemModel(c_engine, c_entity){}

      virtual ~CIridiaTrackingSystemRectangularModel() {}

      virtual Real GetHalfEntityXSize() const = 0;
      virtual Real GetHalfEntityYSize() const = 0;

      virtual CEmbodiedEntity* CheckIntersectionWithRay(Real& f_t_on_ray, const CRay3& c_ray) const {
    	  CRay2 cRay (CVector2(c_ray.GetStart().GetX(),c_ray.GetStart().GetY()),CVector2(c_ray.GetEnd().GetX(),c_ray.GetEnd().GetY()));
		  // Translation to have a relative position to the center of the BOX
		  cRay.SetStart(cRay.GetStart()-CVector2(m_cArenaPosition.GetX(),m_cArenaPosition.GetY()));
		  cRay.SetEnd(cRay.GetEnd()-CVector2(m_cArenaPosition.GetX(),m_cArenaPosition.GetY()));
		  // Rotation to have a relative position to the orientation of the BOX
		  CRadians cAngleToDrop,cAngleToRotate;
		  m_cArenaOrientation.ToEulerAngles(cAngleToRotate,cAngleToDrop,cAngleToDrop);
		  cRay.SetStart(CVector2(cRay.GetStart()).Rotate(-cAngleToRotate));
		  cRay.SetEnd(CVector2(cRay.GetEnd()).Rotate(-cAngleToRotate));
		  //DEBUG("RAY : %f %f    %f %f\n\n", cRay.GetStart().GetX(), cRay.GetStart().GetY(), cRay.GetEnd().GetX(), cRay.GetEnd().GetY())

		  // Check for intersection. If start is inside the box, this is the intersection
		  if (cRay.GetStart().GetY() <= GetHalfEntityYSize()
			   && cRay.GetStart().GetY() >= -GetHalfEntityYSize()
			   && cRay.GetStart().GetX() <= GetHalfEntityXSize()
			   && cRay.GetStart().GetX() >= -GetHalfEntityXSize()){
			 f_t_on_ray=0;
			 return (CEmbodiedEntity*)&GetEmbodiedEntity();
		  }
		  if (cRay.GetLength() == 0)
			  return NULL;

		  /* The Start point is not in the rectangle. Such that we can determine that
		   * for the first intersection :
		   * if startX > halfRectangleX => possible intersection with right part of the rectangle
		   * else if startX < -halfRectangleX => possible intersection with left part of the rectangle
		   *
		   * if startY > halfRectangleY => possible intersection with top part of the rectangle
		   * else if startY < -halfRectangleY => possible intersection with bottom part of the rectangle
		   *
		   * We know that at least one of theses condition will be ok and max 2.
		   * Once an intersection has been found, the other condition cannot lead to another intersection point.
		   */

		  // ray line equation is Y = CST + (endY - startY)/(endX-startX) * X
		  // let's call (endY - startY)/(endX-startX) the angular coefficient
		  // the constant is equal to the value while replacing a point (lets say starting point) in the equation
		  // fCST = startY - startX * fAngularCoef;

		  Real fAngularCoef = (cRay.GetEnd().GetY() - cRay.GetStart().GetY()) / (cRay.GetEnd().GetX() - cRay.GetStart().GetX());
		  Real fCST = cRay.GetStart().GetY() - cRay.GetStart().GetX() * fAngularCoef;
		  if (cRay.GetStart().GetX() > GetHalfEntityXSize()){
			  //X=sizeX/2
			  //=> intersection is Y = CST + fAngularCoef * sizeX/2
			  Real fY = fCST + fAngularCoef * GetHalfEntityXSize();
			  // If in the bound of the rectangle
			  if (fY <= GetHalfEntityYSize() && fY >= -GetHalfEntityYSize()){
				  //real intersection
				  f_t_on_ray = CRay2(CVector2(cRay.GetStart().GetX(),cRay.GetStart().GetY()),CVector2(GetHalfEntityXSize(), fY)).GetLength() / (cRay.GetLength());
				  return (CEmbodiedEntity*)&GetEmbodiedEntity();
			  }
		  }
		  else if (cRay.GetStart().GetX() < - GetHalfEntityXSize()){
			  //X=-sizeX/2
			  Real fY = fCST - fAngularCoef * GetHalfEntityXSize();
			  if (fY <= GetHalfEntityYSize() && fY >= -GetHalfEntityYSize()){
				  f_t_on_ray = CRay2(CVector2(cRay.GetStart().GetX(),cRay.GetStart().GetY()),CVector2(-GetHalfEntityXSize(), fY)).GetLength() / (cRay.GetLength());
				  return (CEmbodiedEntity*)&GetEmbodiedEntity();
			  }
		  }

		  if (cRay.GetStart().GetY() > GetHalfEntityYSize()){
			  //Y=sizeY/2
			  //=> intersection is sizeY/2 = CST + fAngularCoef * X
			  Real fX = ( GetHalfEntityYSize() - fCST ) / fAngularCoef;
			  if (fX <= GetHalfEntityXSize() && fX >= -GetHalfEntityXSize()){
				  //real intersection
				  f_t_on_ray = CRay2(CVector2(cRay.GetStart().GetX(),cRay.GetStart().GetY()),CVector2(fX,GetHalfEntityYSize())).GetLength() / (cRay.GetLength());
				  return (CEmbodiedEntity*)&GetEmbodiedEntity();
			  }
		  }
		  else if (cRay.GetStart().GetY() < - GetHalfEntityYSize()){
			  //Y=-sizeY/2
			  Real fX = ( -GetHalfEntityYSize() - fCST ) / fAngularCoef;
			  if (fX <= GetHalfEntityXSize() && fX >= -GetHalfEntityXSize()){
				  //real intersection
				  f_t_on_ray = CRay2(CVector2(cRay.GetStart().GetX(),cRay.GetStart().GetY()),CVector2(fX,-GetHalfEntityYSize())).GetLength() / (cRay.GetLength());
				  return (CEmbodiedEntity*)&GetEmbodiedEntity();
			  }
		  }
		  return NULL;
      }

   };

}

#endif
