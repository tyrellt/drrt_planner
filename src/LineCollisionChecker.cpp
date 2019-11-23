#include "LineCollisionChecker.h"

bool LineCollisionChecker::checkMotion(const ob::State *s1, const ob::State *s2) const
{
    const ob::RealVectorStateSpace::StateType* s1_2D =
        s1->as<ob::RealVectorStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType* s2_2D =
        s2->as<ob::RealVectorStateSpace::StateType>();

    double x1 = s1_2D->values[0];
    double y1 = s1_2D->values[1];
    double x2 = s2_2D->values[0];
    double y2 = s2_2D->values[1];

    double stateDistance = sqrt((y2-y1)*(y2-y1) + (x2-x1)*(x2-x1));
    double deltaY = y2 - y1;
    double deltaX = x2 - x1;


    double minDist = 100;
    double dist = 0;
    double turtlebotRadius = 0.3;   //TODO: This is also hardcoded in ValidityChecker. put it somewhere they can both access it.

    for( int a = 0; a < obstacles.size(); a = a + 1)
    {
        dist = abs(deltaY*obstacles[a].x - deltaX*obstacles[a].y + x2*y1 - y2*x1) / stateDistance;
        
        if(dist < minDist)
        {
            minDist = dist;
        }
    }
    return minDist;
}