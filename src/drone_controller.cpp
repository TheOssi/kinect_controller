#include "ros/ros.h"
#include "std_msgs/String.h"
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
	int drone_state;
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

void Drone_controller::takeoff() {
	if (drone_state == droneStatus::Landed) {
		system("rostopic pub -1 /ardrone/takeoff std_msgs/Empty");
	}
}

void Drone_controller::land() {
	system("rostopic pub -1 /ardrone/land std_msgs/Empty");
}

void Drone_controller::reset() {
	system("rostopic pub -1 /ardrone/reset std_msgs/Empty");
}

void Drone_controller::navdataCallback(
		const ardrone_autonomy::Navdata& msg) {
	drone_state = msg.state;
}

void Drone_controller::commandCallback(const kinect_controller::droneSpeeds& msg){
	setCommand(msg.sideSpeed,msg.backSpeed,msg.rotateRight,msg.upSpeed);
}

void Drone_controller::setCommand(float roll, float pitch, float yaw,
		float z_velocity) {
	command.linear.x = pitch;
	command.linear.y = roll;
	command.linear.z = z_velocity;
	command.angular.z = yaw;
}
void Drone_controller::sendCommand() {
	if (drone_state == droneStatus::Flying || drone_state == droneStatus::GotoHover || drone_state == droneStatus::Hovering) {
		commandPublisher.publish(command);
	}
}

Drone_controller::Drone_controller(ros::NodeHandle nh) {
	nodeHandle = nh;
	//subscribes to Navdata from Drone
	navdataSubscriber = nodeHandle.subscribe("/ardrone/navdata", 1,
			&Drone_controller::navdataCallback, this);
	commandSubscriber = nodeHandle.subscribe("/drone_command",1,&Drone_controller::commandCallback,this);
	//Publishes Command to ardrone_autonomy
	commandPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel",1);

	//50 hz publish Rate
	ros::Rate loop_rate(50);
	 while (ros::ok())
	  {
		sendCommand();
	    ros::spinOnce();
	    loop_rate.sleep();
	}
}

int main(int argc, char **argv) {
	std_msgs::Empty emp_msg;
	ROS_INFO("Flying ARdrone");

	ros::init(argc, argv, "ARDrone_test");
	ros::NodeHandle node;

	ros::Publisher pub_empty;
	Drone_controller dc(node);

}

