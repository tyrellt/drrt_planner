#include "ValidityChecker.h"


ValidityChecker::ValidityChecker(const ob::SpaceInformationPtr& si) :
    ob::StateValidityChecker(si) 
{
    //int argc;
	//char** argv;
	//ros::init(argc, argv, "models_getter");
	//ros::NodeHandle nh;
	//ros::Subscriber sub = nh.subscribe("/gazebo/model_states",1, &ValidityChecker::gazebo_callback, this);


    //test obstacles
    Obstacle obstacle1(2.0, 2.0, 1.0);
    Obstacle obstacle2(3.0, 4.0, 0.25);
    Obstacle obstacle3(4.0, 3.0, 0.25);
    obstacles.push_back(obstacle1);
    obstacles.push_back(obstacle2);
    obstacles.push_back(obstacle3);
}

void ValidityChecker::gazebo_callback(const gazebo_msgs::ModelStates& msg) {
    std::string r_100 = "100";
    std::string r_75 = "75";
    std::string r_50 = "50";
    std::string r_25 = "25";
    for(int i = 0; i < msg.name.size(); i++){
        std::string model_name = msg.name[i];
        if(model_name.find("obs") != std::string::npos){
            float r;
            if(model_name.find(r_100) != std::string::npos){
            r = 1.0;
            }
            else if(model_name.find(r_75) != std::string::npos){
            r = .75;
            }
            else if(model_name.find(r_50) != std::string::npos){
            r = .5;
            }
            else if(model_name.find(r_25) != std::string::npos){
            r = .25;
            }
            Obstacle obs(msg.pose[i].position.x, msg.pose[i].position.y, r);
            obstacles.push_back(obs);
        }
    }
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
