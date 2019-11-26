#include "ValidityChecker.h"


ValidityChecker::ValidityChecker(const ob::SpaceInformationPtr& si) :
    ob::StateValidityChecker(si) 
{
    // Initialize ros node that subscribes to obstacle data
    // create node handle
    // subscribe to gazebo model getter thing



    //test obstacles
    Obstacle obstacle1(2.0, 2.0, 1.0);
    Obstacle obstacle2(3.0, 4.0, 0.25);
    Obstacle obstacle3(4.0, 3.0, 0.25);
    obstacles.push_back(obstacle1);
    obstacles.push_back(obstacle2);
    obstacles.push_back(obstacle3);

    

}

void ValidityChecker::gazebo_callback(const gazebo_msgs::ModelStates& msg) {
    //read in the message that we're subscribing to.
    //place obstacle data in the vector of Obstacles

}


// Returns whether the given state's position overlaps the
// circular obstacle
bool ValidityChecker::isValid(const ob::State* state) const
{
    bool result = this->clearance(state) > 0.0;
    if (!result) 
        std::cout << "collision!\n";
    
    return result;
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
    double turtlebotRadius = 0.3;

    for( int a = 0; a < obstacles.size(); a = a + 1)
    {
        dist = sqrt((x-obstacles[a].x)*(x-obstacles[a].x) + (y-obstacles[a].y)*(y-obstacles[a].y)) - obstacles[a].radius - turtlebotRadius;
        if(dist < minDist)
        {
            minDist = dist;
        }
    }
    return minDist;
}
