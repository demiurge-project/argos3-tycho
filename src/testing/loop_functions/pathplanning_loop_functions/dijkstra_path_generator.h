/*
 * CDijkstraPathGenerato.h
 *
 *  Created on: 22 août 2014
 *      Author: bernard
 */

#ifndef CDIJKSTRAPATHGENERATOR_H_
#define CDIJKSTRAPATHGENERATOR_H_

#include "path_generator.h"
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>

namespace argos{

class CDijkstraPathGenerator : public CPathGenerator {
public:
	CDijkstraPathGenerator():CPathGenerator(), m_fRobotBodyRadius(0){}
	virtual ~CDijkstraPathGenerator(){};
	virtual void Init(TConfigurationNode& t_node);
	virtual std::vector<CVirtualSensorDataPosition2D> GeneratePathFromTo(CVirtualSensorDataPosition2D c_from,CVirtualSensorDataPosition2D c_to);


protected:
	/*
	 * Generate the graph used for the creation of path
	 */
	void BuildWaypointsGraph();
	/*
	 * Dijkstra algorithm in the graph
	 * @param vector of distances between start position and each node of the graph
	 * @param vector of distances between end position and each node of the graph
	 * @return vector of numbers of nodes for the path from start to destination
	 */
	std::vector<UInt32> Dijkstra(std::vector<Real> vecLengthStartToNodes,std::vector<Real> vecLengthNodesToend);

	Real m_fRobotBodyRadius;
};

}

#endif /* CDIJKSTRAPATHGENERATOR_H_ */
