#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf2_msgs/TFMessage.h>
#include <kinect_controller/droneSpeeds.h>
#include "SkeletonPoints.h"
#include "FuzzyController.h"
#include <string>

class KinectController {
private:
	ros::NodeHandle n;
	SkeletonPoints sPoints;
	FuzzyController fController;
	ros::Publisher commandPublisher;
	int count = 0;
public:
	KinectController(ros::NodeHandle &nodeHandle);
	void messageCallback(const tf2_msgs::TFMessage::ConstPtr& msg);
};
