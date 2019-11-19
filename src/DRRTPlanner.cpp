#include <pluginlib/class_list_macros.h>
#include "DRRTPlanner.h"

#include <iostream>
#include <cmath>

//register this planner as a BaseDRRTPlanner plugin
PLUGINLIB_EXPORT_CLASS(drrt_planner::DRRTPlanner, nav_core::BaseGlobalPlanner)

namespace drrt_planner {

//Default Constructor
DRRTPlanner::DRRTPlanner (){

}

DRRTPlanner::DRRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  	initialize(name, costmap_ros);
}

bool isStateValid(const ob::State *state)
{
    // extract the first component of the state and cast it to what we expect
    const auto *vectorState = state->as<ob::RealVectorStateSpace::StateType>();


    // return
    return true;
}


void DRRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
	std::cout << "in DRRTPlanner initialize\n";
	// construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));

    // set the bounds for the R^3
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);

    // create a random start state
    ob::ScopedState<> start(space);
    start.random();

    // create a random goal state
    ob::ScopedState<> goal(space);
    goal.random();

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner(std::make_shared<og::RRTstar>(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

bool DRRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  
							std::vector<geometry_msgs::PoseStamped>& plan ){

	static bool needToReplan = true;
	if (needToReplan) {
		plan.push_back(start);
		float dx = goal.pose.position.x - start.pose.position.x;
		float dy = goal.pose.position.y - start.pose.position.y;

		//normalize
		float euclDistance = hypot(dx, dy);
		int step = 20;
		int increment = euclDistance / step;
		dx /= step;
		dy /= step;
		std::cout << "dx, dy: " << dx << ", " << dy << "\n";

		float angle = 0.0;//atan2(dy, dx);
		tf::Quaternion quat = tf::createQuaternionFromYaw(angle);

		geometry_msgs::PoseStamped last_point = start;
		plan.push_back(start);

	    for (int i = 0; i < step; i++) {
	    	geometry_msgs::PoseStamped next_point = goal;	// inintialize to goal to get header
	    	next_point.pose.position.x = last_point.pose.position.x + dx;
	    	next_point.pose.position.y = last_point.pose.position.y + dy;
	    	
	    	next_point.pose.orientation.x = quat.x();
	    	next_point.pose.orientation.y = quat.y();
	    	next_point.pose.orientation.z = quat.z();
	    	next_point.pose.orientation.w = quat.w();
	    	std::cout << "just added a point\n";
	    	plan.push_back(next_point);
	    	last_point = next_point;


	    }
	    std::cout << "done planning\n";
	   	plan.push_back(goal);
	   	needToReplan = false;
   	}
   	return true;
}
};
