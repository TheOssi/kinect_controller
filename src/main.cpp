#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf2_msgs/TFMessage.h>
#include <string>

using namespace std;

struct point {
	float x;
	float y;
	float z;
};

struct jointType {
	enum {
		rightHand = 0,
		rightElbow = 1,
		rightShoulder = 2,
		torso = 3,
		neck = 4,
		head = 5,
		rightHip = 6,
		rightFoot = 7,
		rightKnee = 8,
		leftHip = 9,
		leftKnee = 10,
		leftFoot = 11,
		leftShoulder = 12,
		leftElbow = 13,
		leftHand = 14
	};
};
jointType jType;

class SkeletonPoints {
private:
	static bool instanceFlag;
	static SkeletonPoints *single;
	point points[14];

	SkeletonPoints() {
		//private constructor
	}
public:
	static SkeletonPoints* getInstance();

	void setJoint(int, float, float, float);
	point getJoint(int);

	~SkeletonPoints() {
		instanceFlag = false;
	}
};

bool SkeletonPoints::instanceFlag = false;
SkeletonPoints* SkeletonPoints::single = NULL;
SkeletonPoints* SkeletonPoints::getInstance() {
	if (!instanceFlag) {
		single = new SkeletonPoints();
		instanceFlag = true;
		return single;
	} else {
		return single;
	}
}

void SkeletonPoints::setJoint(int joint, float x, float y, float z) {
	points[joint].x = x;
	points[joint].y = y;
	points[joint].z = z;
}
point SkeletonPoints::getJoint(int joint) {
	return points[joint];
}

void messageCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
	SkeletonPoints* points;
	points = SkeletonPoints::getInstance();
	float x = msg->transforms[0].transform.translation.x;
	float y = msg->transforms[0].transform.translation.y;
	float z = msg->transforms[0].transform.translation.z;
	string childframe(msg->transforms[0].child_frame_id.c_str());

	//RIGHT HAND
	if (childframe == "right_hand_1") {
		points->setJoint(jType.rightHand, x, y, z);
		//RIGHT ELBOW
	} else if (childframe == "right_elbow_1") {
		points->setJoint(jType.rightElbow, x, y, z);
		//RIGHT SHOULDER
	} else if (childframe == "right_shoulder_1") {
		points->setJoint(jType.rightShoulder, x, y, z);
		//RIGHT HIP
	} else if (childframe == "right_hip_1") {
		points->setJoint(jType.rightHip, x, y, z);
		//RIGHT KNEE
	} else if (childframe == "right_knee_1") {
		points->setJoint(jType.rightElbow, x, y, z);
		//RIGHT FOOT
	} else if (childframe == "right_foot_1") {
		points->setJoint(jType.rightElbow, x, y, z);
		//LEFT HAND
	} else if (childframe == "left_hand_1") {
		points->setJoint(jType.leftHand, x, y, z);
		//LEFT ELBOW
	} else if (childframe == "left_elbow_1") {
		points->setJoint(jType.leftElbow, x, y, z);
		//LEFT SHOULDER
	} else if (childframe == "left_shoulder_1") {
		points->setJoint(jType.leftShoulder, x, y, z);
		//LEFT HIP
	} else if (childframe == "left_hip_1") {
		points->setJoint(jType.leftHip, x, y, z);
		//LEFT KNEE
	} else if (childframe == "left_knee_1") {
		points->setJoint(jType.leftElbow, x, y, z);
		//LEFT FOOT
	} else if (childframe == "left_foot_1") {
		points->setJoint(jType.leftElbow, x, y, z);
	}
	//TORSO
	else if (childframe == "torso_1") {
		points->setJoint(jType.torso, x, y, z);
	}
	//NECK
	else if (childframe == "neck_1") {
		points->setJoint(jType.neck, x, y, z);
	}
	//HEAD
	else if (childframe == "head_1") {
		points->setJoint(jType.head, x, y, z);
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "listener");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/tf", 1000, messageCallback);

	ros::spin();

	return 0;
}

