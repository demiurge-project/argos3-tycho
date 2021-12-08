/*
 * CDijkstraPathGenerato.cpp
 *
 *  Created on: 22 août 2014
 *      Author: bernard
 */

#include "dijkstra_path_generator.h"

namespace argos {

void CDijkstraPathGenerator::Init(TConfigurationNode& t_node){
	TConfigurationNode& arena = GetNode(CSimulator::GetInstance().GetConfigurationRoot(),"arena");
	// Set the size of the robot
	if (NodeExists(arena,"e-puck")){
		m_fRobotBodyRadius = 0.035f;
	}
	if (NodeExists(arena,"foot-bot")){
		if (m_fRobotBodyRadius)
			THROW_ARGOSEXCEPTION("ONLY ONE ROBOT ALLOWED AT THE SAME TIME")
		m_fRobotBodyRadius = 0.085036758f;
	}
	// Build the graph with the corresponding size of the robot
	BuildWaypointsGraph();
}

void CDijkstraPathGenerator::BuildWaypointsGraph(){
	CSpace& cSpace = CSimulator::GetInstance().GetSpace();
	Real fDistFromObjects = m_fRobotBodyRadius * 2.0f;
	std::vector<CVector2> vecPossibleWaypoints;
	m_vecWaypoints = std::vector<CVector2> ();

	CVector2 cArenaPosition;
	CVector2 cVertices[4];

	// ADD all corners of boxes as waypoints
	try{
		CSpace::TMapPerType& m_cBox = cSpace.GetEntitiesByType("box");
		CRadians cAngleToDrop,cAngleToRotate;
		for(CSpace::TMapPerType::iterator it = m_cBox.begin(); it != m_cBox.end(); ++it) {
			CBoxEntity& cBoxEntity = *any_cast<CBoxEntity*>(it->second);
			cBoxEntity.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cAngleToRotate,cAngleToDrop,cAngleToDrop);
			cArenaPosition = CVector2(cBoxEntity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),cBoxEntity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
			cVertices[0] = CVector2(-cBoxEntity.GetSize().GetX()/2 - fDistFromObjects,  cBoxEntity.GetSize().GetY()/2 + fDistFromObjects).Rotate(cAngleToRotate)+cArenaPosition;
			cVertices[1] = CVector2(-cBoxEntity.GetSize().GetX()/2 - fDistFromObjects, -cBoxEntity.GetSize().GetY()/2 - fDistFromObjects).Rotate(cAngleToRotate)+cArenaPosition;
			cVertices[2] = CVector2( cBoxEntity.GetSize().GetX()/2 + fDistFromObjects, -cBoxEntity.GetSize().GetY()/2 - fDistFromObjects).Rotate(cAngleToRotate)+cArenaPosition;
			cVertices[3] = CVector2( cBoxEntity.GetSize().GetX()/2 + fDistFromObjects,  cBoxEntity.GetSize().GetY()/2 + fDistFromObjects).Rotate(cAngleToRotate)+cArenaPosition;
			vecPossibleWaypoints.push_back(cVertices[0]);
			vecPossibleWaypoints.push_back(cVertices[1]);
			vecPossibleWaypoints.push_back(cVertices[2]);
			vecPossibleWaypoints.push_back(cVertices[3]);
		}
	}
	catch (CARGoSException &ex) {}
	// ADD all corners of the box containing a cylinder as waypoints
	try {
		CSpace::TMapPerType& m_cCylinder = cSpace.GetEntitiesByType("cylinder");
		for(CSpace::TMapPerType::iterator it = m_cCylinder.begin(); it != m_cCylinder.end(); ++it) {
			CCylinderEntity& cCylinerEntity = *any_cast<CCylinderEntity*>(it->second);
			cArenaPosition = CVector2(cCylinerEntity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),cCylinerEntity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
			cVertices[0] = CVector2(-cCylinerEntity.GetRadius() - fDistFromObjects,  cCylinerEntity.GetRadius() + fDistFromObjects)+cArenaPosition;
			cVertices[1] = CVector2(-cCylinerEntity.GetRadius() - fDistFromObjects, -cCylinerEntity.GetRadius() - fDistFromObjects)+cArenaPosition;
			cVertices[2] = CVector2( cCylinerEntity.GetRadius() + fDistFromObjects, -cCylinerEntity.GetRadius() - fDistFromObjects)+cArenaPosition;
			cVertices[3] = CVector2( cCylinerEntity.GetRadius() + fDistFromObjects,  cCylinerEntity.GetRadius() + fDistFromObjects)+cArenaPosition;
			vecPossibleWaypoints.push_back(cVertices[0]);
			vecPossibleWaypoints.push_back(cVertices[1]);
			vecPossibleWaypoints.push_back(cVertices[2]);
			vecPossibleWaypoints.push_back(cVertices[3]);
		}
	}
	catch (CARGoSException &ex) {}
	// TODO: Remove all waypoints inside objects
	/*SBoundingBox sBox;
	bool toRemove;
	for (UInt32 i=0; i<vecPossibleWaypoints.size(); ++i){
		toRemove = false;
		sBox.MinCorner=CVector3(vecPossibleWaypoints[i].GetX()-fRobotBodyRadius,vecPossibleWaypoints[i].GetY()-fRobotBodyRadius,0);
		sBox.MinCorner=CVector3(vecPossibleWaypoints[i].GetX()+fRobotBodyRadius,vecPossibleWaypoints[i].GetY()+fRobotBodyRadius,0.15);
		for (UInt32 j=0; j<vecArenaObjects.size(); ++j){
			if (sBox.Intersects(vecArenaObjects[j])){
				toRemove = true;
				break;
			}
		}
		if (!toRemove)
			m_vecWaypoints.push_back(vecPossibleWaypoints[i]);
	}
	vecPossibleWaypoints = m_vecWaypoints;*/
	// Fusion all waypoints to close of another
	bool bFusionOccurs = true;
	while (bFusionOccurs){
		bFusionOccurs = false;
		for (UInt32 i=0; i<vecPossibleWaypoints.size() && !bFusionOccurs; ++i){
			for (UInt32 j=i+1; j<vecPossibleWaypoints.size() && !bFusionOccurs; ++j){
				if ((vecPossibleWaypoints[i]-vecPossibleWaypoints[j]).Length()<fDistFromObjects){
					//new waypoint between the two waypoints
					vecPossibleWaypoints[i] = vecPossibleWaypoints[i]-(vecPossibleWaypoints[i]-vecPossibleWaypoints[j])/2;
					vecPossibleWaypoints.erase(vecPossibleWaypoints.begin()+j);
					bFusionOccurs = true;
				}
			}
		}
	}
	m_vecWaypoints=vecPossibleWaypoints;
	// Got all waypoints, now calculate distance between positions (Fill the matrix of distances)
	m_vecWaypointsLinksMatrix = std::vector<std::vector<Real> >();
	for (UInt32 i=0; i<m_vecWaypoints.size(); ++i){
		m_vecWaypointsLinksMatrix.push_back(std::vector<Real>());
		for (UInt32 j=0; j<m_vecWaypoints.size(); ++j){
			CRay3 cRay (CVector3(m_vecWaypoints[i].GetX(),m_vecWaypoints[i].GetY(),0.01),CVector3(m_vecWaypoints[j].GetX(),m_vecWaypoints[j].GetY(),0.01));
			std::set<CEmbodiedEntity*> setIntersections = GetAllEmbodiedEntityIntersectedByRay(cRay);
			bool bIntersection = false;
			for (std::set<CEmbodiedEntity*>::iterator it=setIntersections.begin(); it!=setIntersections.end() && !bIntersection; ++it){
				// If intersection is an object (not a robot) we got a real intersection
				if ((*it)->GetParent().GetTypeDescription() == "box" || (*it)->GetParent().GetTypeDescription() == "cylinder"){
					bIntersection = true;
				}
			}
			// if no intersection, value is distance, else it is 0
			m_vecWaypointsLinksMatrix[i].push_back(
				!bIntersection
						? (vecPossibleWaypoints[i]-vecPossibleWaypoints[j]).Length()
						: 0);
		}
	}
}

std::vector<CVirtualSensorDataPosition2D> CDijkstraPathGenerator::GeneratePathFromTo(CVirtualSensorDataPosition2D c_from,CVirtualSensorDataPosition2D c_to){
	// Init the solution
	std::vector<CVirtualSensorDataPosition2D> vecWaypoints = std::vector<CVirtualSensorDataPosition2D>();
	// If no objects in the arena, trivial solution
	if (m_vecWaypoints.size() == 0){
		// Directly go to the destination
		LOG << "Direct dest" << std::endl;
		vecWaypoints.push_back(c_to);
		return vecWaypoints;
	}

	// Check if destination is directly reachable (no obstacle between start and end)
	CRay3 cRayStart (CVector3(c_from.XRange,c_from.YRange,0.01),CVector3(c_to.XRange,c_to.YRange,0.01));
	std::set<CEmbodiedEntity*> setIntersections = GetAllEmbodiedEntityIntersectedByRay(cRayStart);
	bool bIntersection = false;
	for (std::set<CEmbodiedEntity*>::iterator it=setIntersections.begin(); it!=setIntersections.end() && !bIntersection; ++it){
		if ((*it)->GetParent().GetTypeDescription() == "box" || (*it)->GetParent().GetTypeDescription() == "cylinder"){
			bIntersection = true;
		}
	}

	// If no intersection between start and end
	if (!bIntersection)
		vecWaypoints.push_back(c_to);
	else {
		// We have typical min flow problem
		// We should try to add the position and destination to the graph and then resolve the problem:
		std::vector<Real> vecLengthStartToNodes;
		CRay3 cRayStart (CVector3(c_from.XRange,c_from.YRange,0.01),CVector3(0,0,0.01));
		std::vector<Real> vecLengthEndToNodes;
		CRay3 cRayEnd (CVector3(0,0,0.02),CVector3(c_to.XRange,c_to.YRange,0.02));
		for (UInt32 i=0; i<m_vecWaypoints.size(); ++i){

			/* Check intersections between start and nodes */
			cRayStart.GetEnd().SetX(m_vecWaypoints[i].GetX());
			cRayStart.GetEnd().SetY(m_vecWaypoints[i].GetY());
			std::set<CEmbodiedEntity*> setIntersections = GetAllEmbodiedEntityIntersectedByRay(cRayStart);
			bool bIntersection = false;
			for (std::set<CEmbodiedEntity*>::iterator it=setIntersections.begin(); it!=setIntersections.end() && !bIntersection; ++it){
				if ((*it)->GetParent().GetTypeDescription() == "box" || (*it)->GetParent().GetTypeDescription() == "cylinder"){
					bIntersection = true;
				}
			}
			vecLengthStartToNodes.push_back(	!bIntersection ? cRayStart.GetLength() : 0);

			/* Check intersections between destinations and nodes */
			cRayEnd.GetStart().SetX(m_vecWaypoints[i].GetX());
			cRayEnd.GetStart().SetY(m_vecWaypoints[i].GetY());
			setIntersections = GetAllEmbodiedEntityIntersectedByRay(cRayEnd);
			bIntersection = false;
			for (std::set<CEmbodiedEntity*>::iterator it=setIntersections.begin(); it!=setIntersections.end() && !bIntersection; ++it){
				if ((*it)->GetParent().GetTypeDescription() == "box" || (*it)->GetParent().GetTypeDescription() == "cylinder"){
					bIntersection = true;
				}
			}
			vecLengthEndToNodes.push_back(	!bIntersection ? cRayStart.GetLength() : 0);
		}
		/* All waypoints ready :
		 * Dijkstra
		 */
		std::vector<UInt32> vecWaypointsNumbers = Dijkstra (vecLengthStartToNodes, vecLengthEndToNodes);

		for (UInt32 i=0; i<vecWaypointsNumbers.size(); ++i){
			vecWaypoints.push_back(CVirtualSensorDataPosition2D(m_vecWaypoints[vecWaypointsNumbers[i]].GetX(),m_vecWaypoints[vecWaypointsNumbers[i]].GetY(),0));
		}
		// add the destination to the points to reach
		vecWaypoints.push_back(c_to);
	}
	return vecWaypoints;
}

std::vector<UInt32> CDijkstraPathGenerator::Dijkstra(std::vector<Real> vecLengthStartToNodes,std::vector<Real> vecLengthNodesToend) {
	Real fInfinity = std::numeric_limits<Real>::max();
	std::vector<Real> vecLengthTo;
	std::vector<SInt32> fromNode;
	std::set<UInt32> toVisit;
	for (UInt32 i=0; i<m_vecWaypoints.size(); ++i) {
		vecLengthTo.push_back(vecLengthStartToNodes[i] ? vecLengthStartToNodes [i] : fInfinity);
		fromNode.push_back(vecLengthStartToNodes[i] ? m_vecWaypoints.size() : -1);
		if (vecLengthStartToNodes[i])
			toVisit.insert(i);
	}
	while (!toVisit.empty()){
		// choose min from toVisit
		std::set<UInt32>::iterator itMin = toVisit.begin();
		for (std::set<UInt32>::iterator it = toVisit.begin() ++; it != toVisit.end(); ++it){
			if (vecLengthTo[*it] < vecLengthTo[*itMin])
				itMin = it;
		}
		// remove it from toVisit
		UInt32 unNode = *itMin;
		toVisit.erase(itMin);
		// for all nodes add toVisitDistance+distanceToNextNode if < actual value of vecLengthTo
		for (UInt32 i=0; i<m_vecWaypoints.size(); ++i) {
			if (m_vecWaypointsLinksMatrix[unNode][i]
					&& (vecLengthTo[unNode] + m_vecWaypointsLinksMatrix[unNode][i] < vecLengthTo[i])){
				toVisit.insert(i);
				vecLengthTo[i] = vecLengthTo[unNode] + m_vecWaypointsLinksMatrix[unNode][i];
				// if so, append theses nodes to toVisit, update fromNode vector
				fromNode[i] = unNode;
			}
		}
	}
	// from all nodes choose the one that minimize vecLengthTo+vecLengthNodesToEnd
	Real fLength = fInfinity;
	UInt32 unPosSmallest;
	for (UInt32 i=0; i<vecLengthTo.size(); ++i){
		if (vecLengthNodesToend[i]
				 && (vecLengthTo[i] + vecLengthNodesToend[i] < fLength)){
			fLength = vecLengthTo[i] + vecLengthNodesToend[i];
			unPosSmallest = i;
		}
	}
	// No possible path to the destination !
	if (fLength == fInfinity)
		return std::vector<UInt32>();
	// else build the inverted vector
	std::vector<UInt32> vecListReverseWaypoints;
	while (unPosSmallest != vecLengthTo.size()){
		vecListReverseWaypoints.push_back(unPosSmallest);
		unPosSmallest = fromNode[unPosSmallest];
	}
	// and revert it
	std::vector<UInt32> vecListWaypoints;
	for (UInt32 i = 0; i<vecListReverseWaypoints.size(); ++i)
		vecListWaypoints.push_back(vecListReverseWaypoints[vecListReverseWaypoints.size()-1-i]);
	return vecListWaypoints;
}

}

REGISTER_PATH_GENERATOR(CDijkstraPathGenerator, "dijkstra_path_generator")
