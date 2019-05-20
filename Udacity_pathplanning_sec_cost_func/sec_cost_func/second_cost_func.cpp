/*
In most situations, a single cost function will not be sufficient to produce complex vehicle behavior. 
In this quiz, we'd like you to implement one more cost function in C++. 
We will use these two C++ cost functions later in the lesson. 
The goal with this quiz is to create a cost function that would make the vehicle drive in the fastest possible lane, given several behavior options. 
We will provide the following four inputs to the function:

Target speed: Currently set as 10 (unitless), the speed at which you would like the vehicle to travel.
Intended lane: the intended lane for the given behavior. 
For PLCR, PLCL, LCR, and LCL, this would be the one lane over from the current lane.
Final lane: the immediate resulting lane of the given behavior. 
For LCR and LCL, this would be one lane over.
A vector of lane speeds, based on traffic in that lane: {6, 7, 8, 9}.
Your task in the implementation will be to create a cost function that satisifes:

The cost decreases as both intended lane and final lane are higher speed lanes.
The cost function provides different costs for each possible behavior: KL, PLCR/PLCL, LCR/LCL.
The values produced by the cost function are in the range 0 to 1.
You can implement your solution in cost.cpp below.
*/

#include <iostream>
#include <vector>
#include "cost.h"

using std::cout;
using std::endl;

int main() {
	// Target speed of our vehicle
	int target_speed = 10;

	// Lane speeds for each lane
	std::vector<int> lane_speeds = { 6, 7, 8, 9 };

	// Test cases used for grading - do not change.
	double cost;
	cout << "Costs for (intended_lane, final_lane):" << endl;
	cout << "---------------------------------------------------------" << endl;
	cost = inefficiency_cost(target_speed, 3, 3, lane_speeds);
	cout << "The cost is " << cost << " for " << "(3, 3)" << endl;
	cost = inefficiency_cost(target_speed, 2, 3, lane_speeds);
	cout << "The cost is " << cost << " for " << "(2, 3)" << endl;
	cost = inefficiency_cost(target_speed, 2, 2, lane_speeds);
	cout << "The cost is " << cost << " for " << "(2, 2)" << endl;
	cost = inefficiency_cost(target_speed, 1, 2, lane_speeds);
	cout << "The cost is " << cost << " for " << "(1, 2)" << endl;
	cost = inefficiency_cost(target_speed, 1, 1, lane_speeds);
	cout << "The cost is " << cost << " for " << "(1, 1)" << endl;
	cost = inefficiency_cost(target_speed, 0, 1, lane_speeds);
	cout << "The cost is " << cost << " for " << "(0, 1)" << endl;
	cost = inefficiency_cost(target_speed, 0, 0, lane_speeds);
	cout << "The cost is " << cost << " for " << "(0, 0)" << endl;

	return 0;
}