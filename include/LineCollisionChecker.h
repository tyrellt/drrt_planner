#ifndef LINECOLLISIONCHECKER_H
#define LINECOLLISIONCHECKER_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class LineCollisionChecker : public ob::MotionValidator
{
public:
    bool checkMotion(const ob::State *s1, const ob::State *s2) const;
};





#endif