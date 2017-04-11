#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <kinect_controller/droneSpeeds.h>
#include <cstdint>

struct droneStatus{
	enum{
		Emergency = 0,
		Inited    = 1,
		Landed    = 2,
		Flying    = 3,
		Hovering  = 4,
		Test      = 5,
		TakingOff = 6,
		GotoHover = 7,
		Landing   = 8,
		Looping   = 9
	};
};

class Drone_controller {
private:
	bool landed;
	int drone_state=2;
	ros::NodeHandle nodeHandle;
	ros::Subscriber navdataSubscriber;
	ros::Subscriber commandSubscriber;
	ros::Publisher commandPublisher;
	geometry_msgs::Twist command;
public:
	Drone_controller(ros::NodeHandle);
	void takeoff();
	void land();
	void reset();
	void navdataCallback(const ardrone_autonomy::Navdata& msg);
	void commandCallback(const kinect_controller::droneSpeeds& msg);
	void setCommand(float roll, float pitch, float yaw, float z_velocity);
	void sendCommand();
};
