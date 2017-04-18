#include "DroneController.h"

void Drone_controller::takeoff() {
	if (drone_state == droneStatus::Landed) {
		ROS_INFO("Drone Takeoff");
		system("rostopic pub -1 /ardrone/takeoff std_msgs/Empty");
	}
}

void Drone_controller::land() {
	ROS_INFO("Landing");
	system("rostopic pub -1 /ardrone/land std_msgs/Empty");
}

void Drone_controller::reset() {
	system("rostopic pub -1 /ardrone/reset std_msgs/Empty");
}

void Drone_controller::navdataCallback(const ardrone_autonomy::Navdata& msg) {
	drone_state = msg.state;
}

void Drone_controller::commandCallback(
		const kinect_controller::droneSpeeds& msg) {
	setCommand(msg.sideSpeed, msg.backSpeed, msg.rotateRight, msg.upSpeed);
}

void Drone_controller::setCommand(float roll, float pitch, float yaw,
		float z_velocity) {
	command.linear.x = -1.0 * pitch;
	command.linear.y = roll;
	command.linear.z = z_velocity;
	command.angular.z = yaw;
}
void Drone_controller::sendCommand() {
	if (drone_state == droneStatus::Landed && command.linear.z >= 0.70) {
		takeoff();
	} else if (drone_state == droneStatus::Flying
			|| drone_state == droneStatus::GotoHover
			|| drone_state == droneStatus::Hovering) {
		if (command.linear.z <= -0.8) {
			land();
		} else {
			commandPublisher.publish(command);
		}
	}
}

Drone_controller::Drone_controller(ros::NodeHandle nh) {
	nodeHandle = nh;
	//subscribes to Navdata from Drone
	navdataSubscriber = nodeHandle.subscribe("/ardrone/navdata", 1,
			&Drone_controller::navdataCallback, this);
	commandSubscriber = nodeHandle.subscribe("/drone_command", 1,
			&Drone_controller::commandCallback, this);
	//Publishes Command to ardrone_autonomy
	commandPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel",
			1);

	//50 hz publish Rate
	ros::Rate loop_rate(100);
	while (ros::ok()) {
		sendCommand();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char **argv) {
	std_msgs::Empty emp_msg;

	ros::init(argc, argv, "ARDrone_test");
	ros::NodeHandle node;

	ros::Publisher pub_empty;
	Drone_controller dc(node);
	dc.land();

}

