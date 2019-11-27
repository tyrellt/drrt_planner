#ifndef VALIDITYCHECKER_H
#define VALIDITYCHECKER_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <gazebo_msgs/ModelStates.h>
#include <ros/ros.h>
#include <ros/console.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct Obstacle {
    Obstacle(float x, float y, float radius) {
        this->x = x;
        this->y = y;
        this->radius = radius;
    }
    float x;
    float y;
    float radius;
};

class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si);

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const;

    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State* state) const;

    void readObstacles(const gazebo_msgs::ModelStates& msg);

    std::vector<Obstacle> obstacles;
};

#endif