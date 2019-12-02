#include <pluginlib/class_list_macros.h>
#include "DRRTPlanner.h"
#include "ValidityChecker.h"
#include "drawPlannerData.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64.h>

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

void DRRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    space = std::make_shared<ob::RealVectorStateSpace>(2);

	// set the bounds for the R^2
	ob::RealVectorBounds bounds(2);
	bounds.setLow(0);
	bounds.setHigh(10);

	space->setBounds(bounds);
	// construct an instance of  space information from this state space
	si = std::make_shared<ob::SpaceInformation>(space);
	vc = new ValidityChecker(si);
	// set state validity checking for this space
	si->setStateValidityChecker(ob::StateValidityCheckerPtr(vc));
	
}


bool DRRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  
							std::vector<geometry_msgs::PoseStamped>& plan )
{
	static bool needToReplan = true;
	
	
	// Option 1:------------------------------------------------
	// check for input from user

	// if user indicates that they want to place an obstacle
		// get path
		// spawn an obstacle along the path in front of the robot somewhere
		// call ValidityChecker::readObstacles()
		// set needToReplan to true

	// Option 2:------------------------------------------------
	// Spawn obstacle using a separate ros node. The node could be run from the command line
	// subscribe to gazebo/model_states
	// if number of models in the environment changes
		// run ValidityChecker::readObstacles()

	

	if (needToReplan) {
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
	    auto planner = std::make_shared<og::RRTstar>(si);

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
    	    og::PathGeometric rrtStarPath( dynamic_cast< const og::PathGeometric& >(*pdef->getSolutionPath()));
    	    std::cout << "Found solution:" << std::endl;

			//Code to plot tree and solution path.
			ob::PlannerData data(si);
    		planner->getPlannerData(data);
    		og::PathGeometric* spath = pdef->getSolutionPath()->as<og::PathGeometric>();
    		drawGraph(data, spath);
			
    	    // print the path to screen
    	    rrtStarPath.print(std::cout);
    	    
    	    plan.push_back(start);
    	    for (int i = 1; i < rrtStarPath.getStateCount() - 1; i++) {	//don't add start and goal states in loop
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
		 
	}

   	return true;
}
};
