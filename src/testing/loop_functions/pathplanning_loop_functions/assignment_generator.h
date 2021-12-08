/*
 * assignment_generator.h
 *
 *  Created on: 22 août 2014
 *      Author: bernard
 */

#ifndef ASSIGNMENT_GENERATOR_H_
#define ASSIGNMENT_GENERATOR_H_

#include <argos3/plugins/robots/generic/control_interface/ci_destination_virtual_sensor.h>
#include <vector>
#include <argos3/core/utility/plugins/factory.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

#include "path_generator.h"

namespace argos {

class CAssignmentGenerator {
public :
	CAssignmentGenerator():
		m_bSharedDestination(true),
		m_cPathGenerator(NULL){}

	virtual ~CAssignmentGenerator(){};

	virtual void Init(TConfigurationNode& t_node){
		/* Init the path generator used for the assignment */
	    std::string strParam = "dijkstra_path_generator";
	    GetNodeAttributeOrDefault(t_node, "path", strParam, strParam);
	    m_cPathGenerator = CFactory<CPathGenerator>::New(strParam);
	    m_cPathGenerator->Init(t_node);

	    GetNodeAttribute(t_node, "sharedDestination", m_bSharedDestination);
	}
	/*
	 * Abstract method that should make the decision of the assigment of the robots.
	 * @param Vector of position of the robots
	 * @param Vector of destinations for the robots
	 * @return Vector of the number of the destination for the i^th robot
	 */
	virtual std::vector<UInt32> MakeDecisionAssignmentPath(std::vector<CVirtualSensorDataPosition2D> vec_positions,std::vector<CVirtualSensorDataPosition2D> vec_destinations) = 0;

	CPathGenerator* GetPathGenerator(){
		return m_cPathGenerator;
	}
protected:
	bool m_bSharedDestination;
	CPathGenerator* m_cPathGenerator;
};

}

/**
 * Registers a assignment function class inside ARGoS.
 * You must register your assignment function class for ARGoS to be able to recognize it.
 * This statement must be included in a .cpp file. It can't be in a header.
 */
#define REGISTER_ASSIGNMENT_FUNCTIONS(CLASSNAME, LABEL) \
   REGISTER_SYMBOL(CAssignmentGenerator,                \
                   CLASSNAME,                     \
                   LABEL,                         \
                   "undefined",                   \
                   "undefined",                   \
                   "undefined",                   \
                   "undefined",                   \
                   "undefined")
#endif /* ASSIGNMENT_GENERATOR_H_ */
