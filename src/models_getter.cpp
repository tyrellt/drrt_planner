#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <vector>

/*****************gazebo_msg::ModelStates type hierarchy*****************
gazebo_msg::ModelStates
    std::vector<std::string> name
    std::vector<geometry_msgs::Pose> pose
        geometry_msgs::Point position
            float64 x
            float64 y
            float64 z
        geometry_msgs::Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
Example: get x-axis angular position from i-th ModelStates object:
_Float64 x_ang_pos = obj.pose[i].orientation.x;
*************************************************************************/
void callback(const gazebo_msgs::ModelStates& msg){

    std::vector<std::string> model_names;
    std::vector<std::vector<_Float64>> model_positions;
    std::vector<_Float64> turtlebot_position;
    //std::vector<std::vector<_Float64>> kwn_obs_positions;
    //std::vector<std::vector<_Float64>> unkwn_obs_positions;
    std::vector<int> model_radii;

    std::string r_100 = "100";
    std::string r_75 = "75";
    std::string r_50 = "50";
    std::string r_25 = "25";

    printf("there are %d objects in the world\n", msg.name.size());
    for(int i = 0; i < msg.name.size(); i++){
        std::string model_name = msg.name[i];
        if(model_name.find("obs") != std::string::npos){
            int r;
            if(model_name.find(r_100) != std::string::npos){
            r = 100;
            }
            else if(model_name.find(r_75) != std::string::npos){
            r = 75;
            }
            else if(model_name.find(r_50) != std::string::npos){
            r = 50;
            }
            else if(model_name.find(r_25) != std::string::npos){
            r = 25;
            }
            //geometry_msgs::Point model_position = msg.pose[i].position;
            std::vector<_Float64> model_coords;
            model_coords.push_back(msg.pose[i].position.x);
            model_coords.push_back(msg.pose[i].position.y);

            model_names.push_back(msg.name[i]);
            model_positions.push_back(model_coords);
            model_radii.push_back(r);

            printf("%s ", model_name.c_str());
            printf("x: %f ", msg.pose[i].position.x);
            printf("y: %f ", msg.pose[i].position.y);
            printf("r: %d\n", r);
        }
        else if(model_name.find("turtlebot") != std::string::npos){
            turtlebot_position.push_back(msg.pose[i].position.x);
            turtlebot_position.push_back(msg.pose[i].position.y);
        }
    }
    printf("\n");

    printf("names size: %d\n", model_names.size());
    for(std::string name : model_names){
        printf("%s | ", name.c_str());
    }
    printf("\n");

    printf("positions size: %d\n", model_positions.size());
    for(std::vector<_Float64> coords : model_positions){
        printf("x: %f, y: %f | ", coords[0], coords[1]);
    }
    printf("\n");

    printf("radii size: %d\n", model_radii.size());
    for(int r : model_radii){
        printf("%d | ", r);
    }
    printf("\n");

    printf("turtlebot at: ");
    for(_Float64 coord : turtlebot_position){
        printf("%f ", coord);
    }
    printf("\n\n\n");

    //ROS_INFO("Name: [%s]", msg.name);
    //ROS_INFO("Pose: [%s]", msg.pose);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "models_getter");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 1, callback);
    ros::spin();
    return 0;
}
