 #ifndef DRRTPLANNER_H
 #define DRRTPLANNER_H

/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf/tf.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>

#include "ValidityChecker.h"

using std::string;

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace drrt_planner {

class DRRTPlanner : public nav_core::BaseGlobalPlanner {
public:

DRRTPlanner();
DRRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

/** overridden classes from interface nav_core::BaseGlobalPlanner **/
void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
bool makePlan(const geometry_msgs::PoseStamped& start,
              const geometry_msgs::PoseStamped& goal,
              std::vector<geometry_msgs::PoseStamped>& plan
             );

//std::shared_ptr<og::RRTstar> planner;
std::shared_ptr<ob::ProblemDefinition> pdef;
std::shared_ptr<ob::RealVectorStateSpace> space;
std::shared_ptr<ob::SpaceInformation> si;

ros::Subscriber collisionSub;

ValidityChecker* vc;

int numObstacles;

};	// end class DRRTPlanner
};	// end namspace drrt_planner
#endif
