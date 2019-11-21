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
	// construct the state space we are planning in
    

    

    
}

bool DRRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  
							std::vector<geometry_msgs::PoseStamped>& plan ){

	static bool needToReplan = true;
	if (needToReplan) {
		auto space(std::make_shared<ob::RealVectorStateSpace>(2));

	    // set the bounds for the R^2
	    ob::RealVectorBounds bounds(2);
	    bounds.setLow(-5);
	    bounds.setHigh(5);

	    space->setBounds(bounds);

	    // construct an instance of  space information from this state space
	    auto si(std::make_shared<ob::SpaceInformation>(space));

	    // set state validity checking for this space
	    si->setStateValidityChecker(isStateValid);
		
	    // create a random start state
	    ob::ScopedState<ob::RealVectorStateSpace> treeStart(space);
	    treeStart[0] = start.pose.position.x;
	    treeStart[1] = start.pose.position.y;
	    std::cout << "start state: " << treeStart[0] << ", " << treeStart[1] << std::endl;
	    
	    ob::ScopedState<ob::RealVectorStateSpace> treeGoal(space);
		treeGoal[0] = goal.pose.position.x;
	    treeGoal[1] = goal.pose.position.y;    
		std::cout << "goal state: " << treeGoal[0] << ", " << treeGoal[1] << std::endl;

		// create a problem instance
	    pdef = std::make_shared<ob::ProblemDefinition>(si);

	    // set the start and goal states
	    double threshold = 0.5;
	    pdef->setStartAndGoalStates(treeStart, treeGoal, threshold);

	    // initialize planner for the defined space
	    planner = std::make_shared<og::RRTstar>(si);

	    // set the problem we are trying to solve for the planner
	    planner->setProblemDefinition(pdef);

	    // perform setup steps for the planner
	    planner->setup();

	    // print the settings for this space
	    std::cout << "\n\n\n\n\nspace settings:\n\n\n";
	    si->printSettings(std::cout);

	    std::cout << "\n\n\n\n\nproblem settings:\n\n\n";
	    // print the problem settings
	    pdef->print(std::cout);
	
		std::cout << "\n\n\n\nsolving...\n\n";
		// attempt to solve the problem within one second of planning time
    	ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);
	
	    	
    	if (solved)
    	{
    	    // get the goal representation from the problem definition (not the same as the goal state)
    	    // and inquire about the found path
    	    og::PathGeometric rrtStarPath( dynamic_cast< const og::PathGeometric& >( *pdef->getSolutionPath()));
    	    std::cout << "Found solution:" << std::endl;
			
    	    // print the path to screen
    	    rrtStarPath.print(std::cout);
    	    
    	    plan.push_back(start);
    	    for (int i = 1; i < rrtStarPath.getStateCount() - 1; i++) {	//don't add start and goal states in loop
    	    	std::cout << "in conversion loop!\n";
    	    	geometry_msgs::PoseStamped nav_pose = goal;	//intialize so we have all the correct header info
    	    	auto currentState = rrtStarPath.getState(i)->as<ob::RealVectorStateSpace::StateType>();
    	    	nav_pose.pose.position.x = currentState->values[0];
    	    	nav_pose.pose.position.y = currentState->values[1];
    	    	plan.push_back(nav_pose);

    	    }
    	    plan.push_back(goal);
    	}
    	else
    	    std::cout << "No solution found" << std::endl;

		needToReplan = false;
		//std::cout << "Plan: \n";
		//for (int i = 0; i < plan.size(); i++) {
		//	std::cout << plan[i].pose.position.x << ", " << plan[i].pose.position.y << std::endl;
		//}
		 
	}









	/* Straight line path example---------------------------------------*/

	// static bool needToReplan = true;
	// if (needToReplan) {
	// 	plan.push_back(start);
	// 	float dx = goal.pose.position.x - start.pose.position.x;
	// 	float dy = goal.pose.position.y - start.pose.position.y;

	// 	//normalize
	// 	float euclDistance = hypot(dx, dy);
	// 	int step = 20;
	// 	int increment = euclDistance / step;
	// 	dx /= step;
	// 	dy /= step;
	// 	std::cout << "dx, dy: " << dx << ", " << dy << "\n";

	// 	float angle = 0.0;//atan2(dy, dx);
	// 	tf::Quaternion quat = tf::createQuaternionFromYaw(angle);

	// 	geometry_msgs::PoseStamped last_point = start;
	// 	plan.push_back(start);

	//     for (int i = 0; i < step; i++) {
	//     	geometry_msgs::PoseStamped next_point = goal;	// inintialize to goal to get header
	//     	next_point.pose.position.x = last_point.pose.position.x + dx;
	//     	next_point.pose.position.y = last_point.pose.position.y + dy;
	    	
	//     	next_point.pose.orientation.x = quat.x();
	//     	next_point.pose.orientation.y = quat.y();
	//     	next_point.pose.orientation.z = quat.z();
	//     	next_point.pose.orientation.w = quat.w();
	//     	std::cout << "just added a point\n";
	//     	plan.push_back(next_point);
	//     	last_point = next_point;


	//     }
	//     std::cout << "done planning\n";
	//    	plan.push_back(goal);
	//    	needToReplan = false;
 //   	}
   	return true;
}
};
