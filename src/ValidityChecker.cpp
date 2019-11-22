#include "ValidityChecker.h"


ValidityChecker::ValidityChecker(const ob::SpaceInformationPtr& si) :
    ob::StateValidityChecker(si) 
{
    // Initialize ros node that subscribes to obstacle data

}


// Returns whether the given state's position overlaps the
// circular obstacle
bool ValidityChecker::isValid(const ob::State* state) const
{
    return this->clearance(state) > 0.0;
}
// Returns the distance from the given state's position to the
// boundary of the circular obstacle.
double ValidityChecker::clearance(const ob::State* state) const
{
    // We know we're working with a RealVectorStateSpace in this
    // example, so we downcast state into the specific type.
    const ob::RealVectorStateSpace::StateType* state2D =
        state->as<ob::RealVectorStateSpace::StateType>();
    // Extract the robot's (x,y) position from its state
    double x = state2D->values[0];
    double y = state2D->values[1];
    // Distance formula between two points, offset by the circle's radius
    double minDist = 100;
    double dist = 0;

    for( int a = 0; a < obstacles.size(); a = a + 1)
    {
        dist = sqrt((x-obstacles[a].x)*(x-obstacles[a].x) + (y-obstacles[a].y)*(y-obstacles[a].y)) - obstacles[a].radius;
        if(dist < minDist)
        {
            minDist = dist;
        }
    }
    return minDist;
}
