/*
 * min_max_assignment_generator.h
 *
 *  Created on: 22 août 2014
 *      Author: bernard
 */

#ifndef MIN_MAX_ASSIGNMENT_GENERATOR_H_
#define MIN_MAX_ASSIGNMENT_GENERATOR_H_

#include <iomanip>
#include "assignment_generator.h"
#include "hungarian/hungarian.hpp"

namespace argos {

class CMinMaxAssignmentGenerator : public CAssignmentGenerator {
public:
	CMinMaxAssignmentGenerator(){}
	/*
	 * Make the decision of the assigment of the robots.
	 * @param Vector of position of the robots
	 * @param Vector of destinations for the robots
	 * @return Vector of the number of the destination for the i^th robot
	 */
	virtual std::vector<UInt32> MakeDecisionAssignmentPath(std::vector<CVirtualSensorDataPosition2D> vec_positions,std::vector<CVirtualSensorDataPosition2D> vec_destinations);
protected:
	void DisplayMatrix(std::vector<std::vector<Real>> matrix);
	void DisplayMatrix(std::vector<std::vector<UInt32>> matrix);
	void DisplayMatrix(Hungarian::Matrix matrix);
	void DisplayVector(std::vector<CVirtualSensorDataPosition2D> vector);
	void DisplayVector(std::vector<UInt32> vector);
};

}

#endif /* MIN_MAX_ASSIGNMENT_GENERATOR_H_ */
