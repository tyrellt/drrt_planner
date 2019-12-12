#include <ros/ros.h>
#include <nav_msgs/Path.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "drrtPathNode");
	{
		ros::NodeHandle n;
    	ros::Publisher path_pub = n.advertise<nav_msgs::Path>("drrtPath", 1000);
	}
	nav_msgs::Path testPath;
	while(ros::ok())
	{

		ros::spinOnce();
	}
	
	return 0;
}