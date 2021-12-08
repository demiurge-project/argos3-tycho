/*
 * path_generator.h
 *
 *  Created on: 22 août 2014
 *      Author: bernard
 */

#ifndef PATH_GENERATOR_H_
#define PATH_GENERATOR_H_


#include <argos3/plugins/robots/generic/control_interface/ci_destination_virtual_sensor.h>
#include <set>
#include <limits>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>
#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

class CPathGenerator {
public :
	CPathGenerator(){}
	virtual ~CPathGenerator(){};
	virtual void Init(TConfigurationNode& t_node) = 0;
	/*
	 * Abstract method that should return a vector of position that correspond to the next waypoints
	 * to reach to get to the destination (c_to) starting from start (c_from).
	 * This should be reimplemented such that it contains it owns logic to resolve the problem.
	 * @return vector<CVirtualSensorDataPosition2D> : waypoints to reach to get to the destination
	 * @return empty vector if no path feasible between the nodes
	 */
	virtual std::vector<CVirtualSensorDataPosition2D> GeneratePathFromTo (CVirtualSensorDataPosition2D c_from, CVirtualSensorDataPosition2D c_to) = 0;

	/*
	 * Calculate the distance between two nodes by generating the waypoints and summing the distances between them.
	 * @return Real : Sum of distances between the nodes to get to the c_to node
	 */
	virtual Real LengthFromTo(CVirtualSensorDataPosition2D c_from, CVirtualSensorDataPosition2D c_to){
		std::vector<CVirtualSensorDataPosition2D> waypoints = GeneratePathFromTo (c_from, c_to);
		if (waypoints.empty()){
			// No way to reach the destination -> the distance is infinite
			return std::numeric_limits<Real>::max();
		}
		CVirtualSensorDataPosition2D cFrom = CVirtualSensorDataPosition2D(c_from.XRange,c_from.YRange,c_from.Bearing);
		Real fLength= CPathGenerator::LengthBetween(cFrom, waypoints[0]);
		for (UInt32 i=0; i<waypoints.size()-1; ++i){
			fLength += CPathGenerator::LengthBetween (waypoints[i], waypoints[i+1]);
		}
		return fLength;
		/* TODO : take concern of turning time into length of the move */
	}

	/*
	 * Calculate the distance between two points
	 * @return Real : distance between the two points
	 */
	static Real LengthBetween(CVirtualSensorDataPosition2D c_from, CVirtualSensorDataPosition2D c_to){
		return sqrt( (c_from.XRange-c_to.XRange)*(c_from.XRange-c_to.XRange) + (c_from.YRange-c_to.YRange)*(c_from.YRange-c_to.YRange) );
	}

	/*
	 * return the set of all objects intersected by a ray by calling the physicsEngine
	 */
	static std::set<CEmbodiedEntity*> GetAllEmbodiedEntityIntersectedByRay(CRay3 c_ray){
		std::set<CEmbodiedEntity*> vecIntersections;
		CPhysicsEngine::TVector& physicsEngines = CSimulator::GetInstance().GetPhysicsEngines();
		for (UInt32 i=0; i<physicsEngines.size();++i){
			Real fTOnRay = 1.0f;
			CEmbodiedEntity* pcIntersection;
			pcIntersection = GetMaxIntersectionEmbodyEntity(fTOnRay, physicsEngines[i], c_ray);
			while (pcIntersection){
				vecIntersections.insert(pcIntersection);
				CVector3 intersection;
				//DEBUG
				c_ray.GetPoint(intersection, fTOnRay);
				//m_intersectionPoints.push_back(intersection);

				// max bound of the length of the object in the ray
				Real fMaxLengthIntersection = (pcIntersection->GetBoundingBox().MaxCorner-pcIntersection->GetBoundingBox().MinCorner).Length()/c_ray.GetLength();
				if (fTOnRay+fMaxLengthIntersection > 1.0f)
					break; // arrived after end of ray
				c_ray.GetPoint(intersection, fTOnRay+fMaxLengthIntersection);
				c_ray.SetStart(intersection);

			    pcIntersection = GetMaxIntersectionEmbodyEntity(fTOnRay, physicsEngines[i], c_ray);
			}
		}
		return vecIntersections;
	}

	virtual std::vector<CVector2> GetWaypointsPosition(){
		return m_vecWaypoints;
	}

	virtual std::vector<std::vector<Real> > GetWaypointsMatrix(){
		return m_vecWaypointsLinksMatrix;
	}

	/*
	 * Returns the entity with strongest intersection.
	 * Note: This function was create to replicate the previous behaviour of CheckIntersectionWithRay.
	 * With the latest version of ARGoS, CheckIntersectionWithRay works differently as it looks for all entities intersecting with c_ray.
	 */
	static CEmbodiedEntity* GetMaxIntersectionEmbodyEntity(Real& f_t_on_ray, CPhysicsEngine* c_physics_engine, CRay3 c_ray) {
		TEmbodiedEntityIntersectionData t_data;
		c_physics_engine->CheckIntersectionWithRay(t_data, c_ray);
		//Real f_t_on_ray = 1.0f;
		CEmbodiedEntity* cFirstCollision = NULL;
		for (std::vector<SEmbodiedEntityIntersectionItem>::iterator it = t_data.begin(); it != t_data.end(); ++it) {
			if ((it->TOnRay) < f_t_on_ray) {
				f_t_on_ray = (*it).TOnRay;
				cFirstCollision = (*it).IntersectedEntity;
			}
		}
		return cFirstCollision;
	}


protected:
	/* Vector of the nodes of the graph */
	std::vector<CVector2> m_vecWaypoints;
	/* Matrix of connection between nodes */
	std::vector<std::vector<Real> > m_vecWaypointsLinksMatrix;
};

}

/**
 * Registers a assignment function class inside ARGoS.
 * You must register your assignment function class for ARGoS to be able to recognize it.
 * This statement must be included in a .cpp file. It can't be in a header.
 */
#define REGISTER_PATH_GENERATOR(CLASSNAME, LABEL) \
   REGISTER_SYMBOL(CPathGenerator,                \
                   CLASSNAME,                     \
                   LABEL,                         \
                   "undefined",                   \
                   "undefined",                   \
                   "undefined",                   \
                   "undefined",                   \
                   "undefined")



#endif /* PATH_GENERATOR_H_ */
