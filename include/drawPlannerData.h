#pragma once

#include "matplotlibcpp.h"


#include <ompl/base/SpaceInformation.h>
//#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
//#include <ompl/base/objectives/StateCostIntegralObjective.h>
//#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
// For ompl::msg::setLogLevel
//#include "ompl/util/Console.h"

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/PlannerData.h>
//#include <ompl/geometric/planners/bitstar/BITstar.h>
//#include <ompl/geometric/planners/cforest/CForest.h>
//#include <ompl/geometric/planners/fmt/FMT.h>
//#include <ompl/geometric/planners/fmt/BFMT.h>
//#include <ompl/geometric/planners/prm/PRMstar.h>
//#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
//#include <ompl/geometric/planners/rrt/RRTstar.h>
//#include <ompl/geometric/planners/rrt/SORRTstar.h>


//#include<boost/format.hpp>
// For boost program options
//#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
//#include <boost/algorithm/string.hpp>
// For std::make_shared
//#include <memory>

//#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace plt = matplotlibcpp;


void drawGraph(const ob::PlannerData &data, const og::PathGeometric *spath = nullptr);

void getXY(const ob::PlannerDataVertex &v, double &x, double &y);
