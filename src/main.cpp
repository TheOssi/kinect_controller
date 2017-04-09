#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf2_msgs/TFMessage.h>
#include "fl/Headers.h"
#include <string>
#include <kinect_controller/droneSpeeds.h>

using namespace fl;
using namespace std;

struct point {
	float x;
	float y;
	float z;
};

struct resultSet {
	float backwardSpeed;
	float sidewardSpeed;
	float upSpeed;
	float rotationSpeed;
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
	point points[14];

public:

	void setJoint(int, float, float, float);
	point getJoint(int);

};
void SkeletonPoints::setJoint(int joint, float x, float y, float z) {
	points[joint].x = x;
	points[joint].y = y;
	points[joint].z = z;
}
point SkeletonPoints::getJoint(int joint) {
	return points[joint];
}

class FuzzyController {
private:
	Engine* engine;

	InputVariable* sideward;
	InputVariable* up;
	InputVariable* rotation;
	OutputVariable* backwardSpeed;
	OutputVariable* sidewardSpeed;
	OutputVariable* upSpeed;
	OutputVariable* rotationSpeed;

public:
	void init();
	resultSet getFISResult(float, float, float, float);

};

void FuzzyController::init() {
	engine = new Engine;
	engine->setName("input");
	engine->setDescription("");
	InputVariable* backward = new InputVariable;
	backward->setName("backward");
	backward->setDescription("");
	backward->setEnabled(true);
	backward->setRange(-1.000, 1.000);
	backward->setLockValueInRange(false);
	backward->addTerm(
			new Trapezoid("strongForward", -2.000, -2.000, -0.800, -0.400));
	backward->addTerm(
			new Trapezoid("mediumForward", -0.900, -0.600, -0.400, 0.000));
	backward->addTerm(
			new Trapezoid("mediumBackward", 0.000, 0.400, 0.600, 0.900));
	backward->addTerm(
			new Trapezoid("strongBackward", 0.400, 0.800, 2.000, 2.000));
	backward->addTerm(new Trapezoid("zero", -0.600, -0.200, 0.200, 0.600));
	engine->addInputVariable(backward);

	sideward = new InputVariable;
	sideward->setName("sideward");
	sideward->setDescription("");
	sideward->setEnabled(true);
	sideward->setRange(-1.000, 1.000);
	sideward->setLockValueInRange(false);
	sideward->addTerm(
			new Trapezoid("strongLeft", -2.000, -2.000, -0.800, -0.400));
	sideward->addTerm(
			new Trapezoid("mediumLeft", -0.900, -0.600, -0.400, 0.000));
	sideward->addTerm(new Trapezoid("mediumRight", 0.000, 0.400, 0.600, 0.900));
	sideward->addTerm(new Trapezoid("strongRight", 0.400, 0.800, 2.000, 2.000));
	sideward->addTerm(new Trapezoid("zero", -0.600, -0.200, 0.200, 0.600));
	engine->addInputVariable(sideward);

	up = new InputVariable;
	up->setName("up");
	up->setDescription("");
	up->setEnabled(true);
	up->setRange(-1.000, 1.000);
	up->setLockValueInRange(false);
	up->addTerm(new Trapezoid("strongDown", -2.000, -2.000, -0.800, -0.400));
	up->addTerm(new Trapezoid("mediumDown", -0.900, -0.600, -0.400, 0.000));
	up->addTerm(new Trapezoid("mediumUp", 0.000, 0.400, 0.600, 0.900));
	up->addTerm(new Trapezoid("strongUp", 0.400, 0.800, 2.000, 2.000));
	up->addTerm(new Trapezoid("zero", -0.600, -0.200, 0.200, 0.600));
	engine->addInputVariable(up);

	rotation = new InputVariable;
	rotation->setName("rotation");
	rotation->setDescription("");
	rotation->setEnabled(true);
	rotation->setRange(-1.000, 1.000);
	rotation->setLockValueInRange(false);
	rotation->addTerm(
			new Trapezoid("strongLeft", -2.000, -2.000, -0.800, -0.400));
	rotation->addTerm(
			new Trapezoid("mediumLeft", -0.900, -0.600, -0.400, 0.000));
	rotation->addTerm(new Trapezoid("mediumRight", 0.000, 0.400, 0.600, 0.900));
	rotation->addTerm(new Trapezoid("strongRight", 0.400, 0.800, 2.000, 2.000));
	rotation->addTerm(new Trapezoid("zero", -0.600, -0.200, 0.200, 0.600));
	engine->addInputVariable(rotation);

	backwardSpeed = new OutputVariable;
	backwardSpeed->setName("backwardSpeed");
	backwardSpeed->setDescription("");
	backwardSpeed->setEnabled(true);
	backwardSpeed->setRange(-1.000, 1.000);
	backwardSpeed->setLockValueInRange(false);
	backwardSpeed->setAggregation(new UnboundedSum);
	backwardSpeed->setDefuzzifier(new Centroid(100));
	backwardSpeed->setDefaultValue(fl::nan);
	backwardSpeed->setLockPreviousValue(false);
	backwardSpeed->addTerm(
			new Triangle("strongForward", -1.200, -1.000, -0.700));
	backwardSpeed->addTerm(
			new Trapezoid("mediumForward", -1.000, -0.850, -0.600, -0.250));
	backwardSpeed->addTerm(
			new Trapezoid("mediumBackward", 0.250, 0.600, 0.850, 1.000));
	backwardSpeed->addTerm(new Triangle("strongBackward", 0.700, 1.000, 2.200));
	backwardSpeed->addTerm(new Trapezoid("zero", -0.800, -0.500, 0.500, 0.800));
	engine->addOutputVariable(backwardSpeed);

	sidewardSpeed = new OutputVariable;
	sidewardSpeed->setName("sidewardSpeed");
	sidewardSpeed->setDescription("");
	sidewardSpeed->setEnabled(true);
	sidewardSpeed->setRange(-1.000, 1.000);
	sidewardSpeed->setLockValueInRange(false);
	sidewardSpeed->setAggregation(new UnboundedSum);
	sidewardSpeed->setDefuzzifier(new Centroid(100));
	sidewardSpeed->setDefaultValue(fl::nan);
	sidewardSpeed->setLockPreviousValue(false);
	sidewardSpeed->addTerm(new Triangle("strongLeft", -1.200, -1.000, -0.700));
	sidewardSpeed->addTerm(
			new Trapezoid("mediumLeft", -1.000, -0.850, -0.600, -0.250));
	sidewardSpeed->addTerm(
			new Trapezoid("mediumRight", 0.250, 0.600, 0.850, 1.000));
	sidewardSpeed->addTerm(new Triangle("strongRight", 0.700, 1.000, 2.200));
	sidewardSpeed->addTerm(new Trapezoid("zero", -0.800, -0.500, 0.500, 0.800));
	engine->addOutputVariable(sidewardSpeed);

	upSpeed = new OutputVariable;
	upSpeed->setName("upSpeed");
	upSpeed->setDescription("");
	upSpeed->setEnabled(true);
	upSpeed->setRange(-1.000, 1.000);
	upSpeed->setLockValueInRange(false);
	upSpeed->setAggregation(new UnboundedSum);
	upSpeed->setDefuzzifier(new Centroid(100));
	upSpeed->setDefaultValue(fl::nan);
	upSpeed->setLockPreviousValue(false);
	upSpeed->addTerm(new Triangle("strongDown", -1.200, -1.000, -0.700));
	upSpeed->addTerm(
			new Trapezoid("mediumDown", -1.000, -0.850, -0.600, -0.250));
	upSpeed->addTerm(new Trapezoid("mediumUp", 0.250, 0.600, 0.850, 1.000));
	upSpeed->addTerm(new Triangle("strongUp", 0.700, 1.000, 2.200));
	upSpeed->addTerm(new Trapezoid("zero", -0.800, -0.500, 0.500, 0.800));
	engine->addOutputVariable(upSpeed);

	rotationSpeed = new OutputVariable;
	rotationSpeed->setName("rotationSpeed");
	rotationSpeed->setDescription("");
	rotationSpeed->setEnabled(true);
	rotationSpeed->setRange(-1.000, 1.000);
	rotationSpeed->setLockValueInRange(false);
	rotationSpeed->setAggregation(new UnboundedSum);
	rotationSpeed->setDefuzzifier(new Centroid(100));
	rotationSpeed->setDefaultValue(fl::nan);
	rotationSpeed->setLockPreviousValue(false);
	rotationSpeed->addTerm(new Triangle("strongLeft", -1.200, -1.000, -0.700));
	rotationSpeed->addTerm(
			new Trapezoid("mediumLeft", -1.000, -0.850, -0.600, -0.250));
	rotationSpeed->addTerm(
			new Trapezoid("mediumRight", 0.250, 0.600, 0.850, 1.000));
	rotationSpeed->addTerm(new Triangle("strongRight", 0.700, 1.000, 2.200));
	rotationSpeed->addTerm(new Trapezoid("zero", -0.800, -0.500, 0.500, 0.800));
	engine->addOutputVariable(rotationSpeed);

	RuleBlock* ruleBlock = new RuleBlock;
	ruleBlock->setName("");
	ruleBlock->setDescription("");
	ruleBlock->setEnabled(true);
	ruleBlock->setConjunction(new Minimum);
	ruleBlock->setDisjunction(new Maximum);
	ruleBlock->setImplication(new Minimum);
	ruleBlock->setActivation(new General);
	ruleBlock->addRule(
			Rule::parse(
					"if backward is strongForward then backwardSpeed is strongForward",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumForward then backwardSpeed is mediumForward",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumBackward then backwardSpeed is strongBackward",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is strongBackward then backwardSpeed is strongBackward",
					engine));
	ruleBlock->addRule(
			Rule::parse("if backward is zero then backwardSpeed is zero",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if sideward is strongLeft then sidewardSpeed is strongLeft",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if sideward is mediumLeft then sidewardSpeed is mediumLeft",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if sideward is mediumRight then sidewardSpeed is mediumRight",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if sideward is strongRight then sidewardSpeed is strongRight",
					engine));
	ruleBlock->addRule(
			Rule::parse("if sideward is zero then sidewardSpeed is zero",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumBackward then backwardSpeed is strongBackward",
					engine));
	ruleBlock->addRule(
			Rule::parse("if up is strongDown then upSpeed is strongDown",
					engine));
	ruleBlock->addRule(
			Rule::parse("if up is mediumDown then upSpeed is mediumDown",
					engine));
	ruleBlock->addRule(
			Rule::parse("if up is mediumUp then upSpeed is mediumUp", engine));
	ruleBlock->addRule(
			Rule::parse("if up is strongUp then upSpeed is strongUp", engine));
	ruleBlock->addRule(
			Rule::parse("if up is zero then upSpeed is zero", engine));
	ruleBlock->addRule(
			Rule::parse(
					"if rotation is strongLeft then rotationSpeed is strongLeft",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if rotation is mediumLeft then rotationSpeed is mediumLeft",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if rotation is mediumRight then rotationSpeed is mediumRight",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if rotation is strongRight then rotationSpeed is strongRight",
					engine));
	ruleBlock->addRule(
			Rule::parse("if rotation is zero then rotationSpeed is zero",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is strongForward and sideward is strongLeft then backwardSpeed is strongForward and sidewardSpeed is strongLeft and rotationSpeed is strongLeft",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumForward and sideward is mediumLeft then backwardSpeed is mediumForward and sidewardSpeed is mediumLeft and rotationSpeed is mediumLeft",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is strongForward and sideward is strongRight then backwardSpeed is strongForward and sidewardSpeed is strongRight and rotationSpeed is strongRight",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumForward and sideward is mediumRight then backwardSpeed is mediumForward and sidewardSpeed is mediumRight and rotationSpeed is mediumRight",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumForward and sideward is strongRight then backwardSpeed is mediumForward and sidewardSpeed is strongRight and rotationSpeed is strongRight",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumForward and sideward is strongLeft then backwardSpeed is mediumForward and sidewardSpeed is strongLeft and rotationSpeed is strongLeft",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumForward and sideward is zero then backwardSpeed is mediumForward and sidewardSpeed is zero and rotationSpeed is zero",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is strongForward and sideward is zero then backwardSpeed is strongForward and sidewardSpeed is zero and rotationSpeed is zero",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is strongForward and sideward is mediumLeft then backwardSpeed is strongForward and sidewardSpeed is mediumLeft and rotationSpeed is strongLeft",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is strongForward and sideward is mediumRight then backwardSpeed is strongForward and sidewardSpeed is mediumRight and rotationSpeed is strongRight",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumBackward then backwardSpeed is mediumBackward",
					engine));
	engine->addRuleBlock(ruleBlock);
}

resultSet FuzzyController::getFISResult(float back, float side, float upValue,
		float rotateRight) {

	resultSet result;
	//ROS_INFO("%.2f| %.2f | %.2f | %.2f", back, side, upValue, rotateRight);
	InputVariable* backward = engine->getInputVariable("backward");
	backward->setValue(back);
	sideward->setValue(side);
	up->setValue(upValue);
	rotation->setValue(rotateRight);

	engine->process();

	result.backwardSpeed = backwardSpeed->getValue();
	result.sidewardSpeed = sidewardSpeed->getValue();
	result.upSpeed = upSpeed->getValue();
	result.rotationSpeed = rotationSpeed->getValue();

	return result;
}

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

KinectController::KinectController(ros::NodeHandle &nodeHandle) {
	sPoints = SkeletonPoints();
	fController = FuzzyController();
	fController.init();
	commandPublisher = n.advertise<kinect_controller::droneSpeeds>("/drone_command",1);
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
		sPoints.setJoint(jType.rightHand, x, y, z);
		count++;
		//RIGHT ELBOW
	} else if (childframe == "right_elbow_1") {
		sPoints.setJoint(jType.rightElbow, x, y, z);
		count++;
		//RIGHT SHOULDER
	} else if (childframe == "right_shoulder_1") {
		sPoints.setJoint(jType.rightShoulder, x, y, z);
		count++;
		//RIGHT HIP
	} else if (childframe == "right_hip_1") {
		sPoints.setJoint(jType.rightHip, x, y, z);
		count++;
		//RIGHT KNEE
	} else if (childframe == "right_knee_1") {
		sPoints.setJoint(jType.rightElbow, x, y, z);
		count++;
		//RIGHT FOOT
	} else if (childframe == "right_foot_1") {
		sPoints.setJoint(jType.rightElbow, x, y, z);
		count++;
		//LEFT HAND
	} else if (childframe == "left_hand_1") {
		sPoints.setJoint(jType.leftHand, x, y, z);
		count++;
		//LEFT ELBOW
	} else if (childframe == "left_elbow_1") {
		sPoints.setJoint(jType.leftElbow, x, y, z);
		count++;
		//LEFT SHOULDER
	} else if (childframe == "left_shoulder_1") {
		sPoints.setJoint(jType.leftShoulder, x, y, z);
		count++;
		//LEFT HIP
	} else if (childframe == "left_hip_1") {
		sPoints.setJoint(jType.leftHip, x, y, z);
		count++;
		//LEFT KNEE
	} else if (childframe == "left_knee_1") {
		sPoints.setJoint(jType.leftElbow, x, y, z);
		count++;
		//LEFT FOOT
	} else if (childframe == "left_foot_1") {
		sPoints.setJoint(jType.leftElbow, x, y, z);
		count++;
	}
	//TORSO
	else if (childframe == "torso_1") {
		sPoints.setJoint(jType.torso, x, y, z);
		count++;
	}
	//NECK
	else if (childframe == "neck_1") {
		sPoints.setJoint(jType.neck, x, y, z);
		count++;
	}
	//HEAD
	else if (childframe == "head_1") {
		sPoints.setJoint(jType.head, x, y, z);
		count++;
	}
	if (count>=14) {
		//calculate joint diffs
		//might need to smooth Signal
		up = sPoints.getJoint(jType.leftHand).z
				- sPoints.getJoint(jType.leftShoulder).z
				+ sPoints.getJoint(jType.rightHand).z
				- sPoints.getJoint(jType.rightShoulder).z;
		side = sPoints.getJoint(jType.leftHand).z
				- sPoints.getJoint(jType.rightHand).z;
		rotateRight = sPoints.getJoint(jType.rightHand).x
				- sPoints.getJoint(jType.leftHand).x;
		back = sPoints.getJoint(jType.neck).x - sPoints.getJoint(jType.torso).x;

		if (up < 2.0 && up > -2.0 && side < 2.0 && side > -2.0
				&& rotateRight < 2.0 && rotateRight > -2.0 && back < 2.0
				&& back > -2.0) {
			fController.init();
			resultSet resultSpeeds = fController.getFISResult(back, side, up,
					rotateRight);
			commandMessage.backSpeed = resultSpeeds.backwardSpeed;
			commandMessage.rotateRight = resultSpeeds.rotationSpeed;
			commandMessage.sideSpeed = resultSpeeds.sidewardSpeed;
			commandMessage.upSpeed = resultSpeeds.upSpeed;
			ROS_INFO("back: %.2f | rotate: %.2f| side: %.2f |up: %.2f",resultSpeeds.backwardSpeed, resultSpeeds.rotationSpeed,resultSpeeds.sidewardSpeed,resultSpeeds.upSpeed);
			commandPublisher.publish(commandMessage);
		}
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "listener");

	ros::NodeHandle n;

	KinectController controller(n);

	return 0;
}
