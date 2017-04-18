#include "KinectController.h"

using namespace std;
KinectController::KinectController(ros::NodeHandle &nodeHandle) {
	sPoints = SkeletonPoints();
	fController = FuzzyController();
	fController.init();
	commandPublisher = n.advertise<kinect_controller::droneSpeeds>(
			"/drone_command", 1);
	ros::Subscriber sub = n.subscribe("/tf", 1000,
			&KinectController::messageCallback, this);
	ros::spin();
}

void KinectController::messageCallback(
		const tf2_msgs::TFMessage::ConstPtr& msg) {
	float up, rotateRight, back, side;
	float x = msg->transforms[0].transform.translation.x;
	float y = msg->transforms[0].transform.translation.y;
	float z = msg->transforms[0].transform.translation.z;
	string childframe(msg->transforms[0].child_frame_id.c_str());
	kinect_controller::droneSpeeds commandMessage;

	//RIGHT HAND
	if (childframe == "right_hand_1") {
		sPoints.setJoint(jointType::rightHand, x, y, z);
		count++;
		//RIGHT ELBOW
	} else if (childframe == "right_elbow_1") {
		sPoints.setJoint(jointType::rightElbow, x, y, z);
		count++;
		//RIGHT SHOULDER
	} else if (childframe == "right_shoulder_1") {
		sPoints.setJoint(jointType::rightShoulder, x, y, z);
		count++;
		//RIGHT HIP
	} else if (childframe == "right_hip_1") {
		sPoints.setJoint(jointType::rightHip, x, y, z);
		count++;
		//RIGHT KNEE
	} else if (childframe == "right_knee_1") {
		sPoints.setJoint(jointType::rightKnee, x, y, z);
		count++;
		//RIGHT FOOT
	} else if (childframe == "right_foot_1") {
		sPoints.setJoint(jointType::rightFoot, x, y, z);
		count++;
		//LEFT HAND
	} else if (childframe == "left_hand_1") {
		sPoints.setJoint(jointType::leftHand, x, y, z);
		count++;
		//LEFT ELBOW
	} else if (childframe == "left_elbow_1") {
		sPoints.setJoint(jointType::leftElbow, x, y, z);
		count++;
		//LEFT SHOULDER
	} else if (childframe == "left_shoulder_1") {
		sPoints.setJoint(jointType::leftShoulder, x, y, z);
		count++;
		//LEFT HIP
	} else if (childframe == "left_hip_1") {
		sPoints.setJoint(jointType::leftHip, x, y, z);
		count++;
		//LEFT KNEE
	} else if (childframe == "left_knee_1") {
		sPoints.setJoint(jointType::leftElbow, x, y, z);
		count++;
		//LEFT FOOT
	} else if (childframe == "left_foot_1") {
		sPoints.setJoint(jointType::leftElbow, x, y, z);
		count++;
	}
	//TORSO
	else if (childframe == "torso_1") {
		sPoints.setJoint(jointType::torso, x, y, z);
		count++;
	}
	//NECK
	else if (childframe == "neck_1") {
		sPoints.setJoint(jointType::neck, x, y, z);
		count++;
	}
	//HEAD
	else if (childframe == "head_1") {
		sPoints.setJoint(jointType::head, x, y, z);
		count++;
	}
	if (count >= 14) {
		//calculate joint diffs
		//might need to smooth Signal
		up = sPoints.getJoint(jointType::leftHand).z
				- sPoints.getJoint(jointType::leftShoulder).z
				+ sPoints.getJoint(jointType::rightHand).z
				- sPoints.getJoint(jointType::rightShoulder).z;
		side = sPoints.getJoint(jointType::leftHand).z
				- sPoints.getJoint(jointType::rightHand).z;
		rotateRight = sPoints.getJoint(jointType::rightHand).x
				- sPoints.getJoint(jointType::leftHand).x;
		back = sPoints.getJoint(jointType::neck).x
				- sPoints.getJoint(jointType::torso).x;

		//fController.init();
		resultSet resultSpeeds = fController.getFISResult(back, side, up,
				rotateRight);
		commandMessage.backSpeed = resultSpeeds.backwardSpeed * 2.0;
		commandMessage.rotateRight = resultSpeeds.rotationSpeed;
		commandMessage.sideSpeed = resultSpeeds.sidewardSpeed;
		commandMessage.upSpeed = resultSpeeds.upSpeed;
		ROS_INFO("back: %.2f | rotate: %.2f| side: %.2f |up: %.2f",
				resultSpeeds.backwardSpeed, resultSpeeds.rotationSpeed,
				resultSpeeds.sidewardSpeed, resultSpeeds.upSpeed);
		//ROS_INFO("left: %.2f | right: %.2f | rot: %.2f",sPoints.getJoint(jointType::rightHand).x,sPoints.getJoint(jointType::leftHand).x,resultSpeeds.rotationSpeed);

		commandPublisher.publish(commandMessage);

	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "listener");

	ros::NodeHandle n;

	KinectController controller(n);

	return 0;
}
