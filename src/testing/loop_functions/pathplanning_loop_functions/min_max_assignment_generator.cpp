/*
 * min_max_assignment_generator.cpp
 *
 *  Created on: 22 août 2014
 *      Author: bernard
 */

#include "min_max_assignment_generator.h"

namespace argos {

std::vector<UInt32> CMinMaxAssignmentGenerator::MakeDecisionAssignmentPath(std::vector<CVirtualSensorDataPosition2D> vec_positions,std::vector<CVirtualSensorDataPosition2D> vec_destinations){
	/*
	 * If shared destination, solution is not trivial, find the assignment
	 		(shared destination means that any robot can go to any destination, destination are not bounded to
			robots)
	 */
	// Number of rob ots must be equal to number of destinations
	UInt32 unNumberRobot = vec_destinations.size();
	// Init solution vector with default values:
	std::vector<UInt32> vecAssignmentDestination = std::vector<UInt32>();
	for (UInt32 i=0; i< unNumberRobot; ++i) {
		vecAssignmentDestination.push_back(i);
	}

	if (m_bSharedDestination){
		// DEBUG:
		// LOG << "Initial positions: " << std::endl;
		// DisplayVector(vec_positions);
		// LOG << "Destinations: " << std::endl;
		// DisplayVector(vec_destinations);

		// Init matrix for resolution and fill with
		// distance values in cm. (integers)
		Hungarian::Matrix vecCostMatrix;
		for (UInt32 i = 0; i < unNumberRobot; i++) {
			vecCostMatrix.push_back(std::vector<SInt32>());
			for (UInt32 j = 0; j < unNumberRobot; j++) {
				vecCostMatrix[i].push_back((SInt32)(m_cPathGenerator->LengthFromTo(vec_positions[i],vec_destinations[j])*100));
				// LengthFromTo takes into account the obstacles when computing distance between robot and destination
				// alternative:
				// m_cPathGenerator->LengthBetween(vec_positions[i],vec_destinations[j])*100;
			}
		}
		// DEBUG:
		// LOG << "Hungarian matrix: \n";
		// DisplayMatrix(vecCostMatrix);

		Hungarian::Result vecMatrixSolution = Hungarian::Solve(vecCostMatrix, Hungarian::MODE_MINIMIZE_COST);
		// DEBUG:
		// LOG << "cost-matrix:" << std::endl;
		// DisplayMatrix(vecMatrixSolution.cost);

		if (!vecMatrixSolution.success) {
			LOG << "Failed to find solution :(" << std::endl;
			LOG << "Falling back to default id assignment" << std::endl;
		} else {
			LOG << "Found assignment" << std::endl;
			// DisplayMatrix(vecMatrixSolution.assignment);
			// assign output matrix:
			for (UInt32 i = 0; i < unNumberRobot; i++) {
				for (UInt32 j = 0; j < unNumberRobot; j++) {
					if (vecMatrixSolution.assignment[i][j] == 1) {
						vecAssignmentDestination[i] = j;
					}
				}
			}
		}
		// DEBUG:
		//DisplayVector(vecAssignmentDestination);
	}
	return vecAssignmentDestination;
}

/****************************************/
/****************************************/

// HELPER FUNCTIONS
void CMinMaxAssignmentGenerator::DisplayMatrix(std::vector<std::vector<Real>> matrix) {
	for (UInt32 i=0; i< matrix.size(); ++i){
		for (UInt32 j=0; j< matrix[i].size(); ++j){
			LOG << matrix[i][j] << "\t";
		}
		LOG << std::endl;
	}
}

/****************************************/
/****************************************/

void CMinMaxAssignmentGenerator::DisplayMatrix(std::vector<std::vector<UInt32>> matrix) {
	for (UInt32 i=0; i< matrix.size(); ++i){
		for (UInt32 j=0; j< matrix[i].size(); ++j){
			LOG << std::setw(2) << std::setfill('0') << matrix[i][j] << "\t";
		}
		LOG << std::endl;
		//DEBUG("\n")
	}
}

/****************************************/
/****************************************/

void CMinMaxAssignmentGenerator::DisplayMatrix(Hungarian::Matrix matrix) {
	for (UInt32 i=0; i< matrix.size(); ++i){
		for (UInt32 j=0; j< matrix[i].size(); ++j){
			LOG << std::setw(2) << std::setfill('0') << matrix[i][j] << "\t";
		}
		LOG << std::endl;
		//DEBUG("\n")
	}
}

/****************************************/
/****************************************/

void CMinMaxAssignmentGenerator::DisplayVector(std::vector<CVirtualSensorDataPosition2D> vector) {
	for (UInt32 i=0; i< vector.size(); ++i){
		LOG << vector[i].XRange << "," << vector[i].YRange << std::endl;
	}
}

/****************************************/
/****************************************/

void CMinMaxAssignmentGenerator::DisplayVector(std::vector<UInt32> vector) {
	for (UInt32 i=0; i< vector.size(); ++i){
		LOG << vector[i] << " ";
	}
	LOG << std::endl;
}

/****************************************/
/****************************************/

}
REGISTER_ASSIGNMENT_FUNCTIONS(CMinMaxAssignmentGenerator, "min_max_assignment_generator")
