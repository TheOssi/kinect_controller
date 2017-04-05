#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf2_msgs/TFMessage.h>
#include <string>
#include "fl/Headers.h"

using namespace fl;
using namespace std;


struct point {
	float x;
	float y;
	float z;
};

struct resultSet{
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
	static bool instanceFlag;
	static SkeletonPoints *instance;
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
SkeletonPoints* SkeletonPoints::instance = NULL;
SkeletonPoints* SkeletonPoints::getInstance() {
	if (!instanceFlag) {
		instance = new SkeletonPoints();
		instanceFlag = true;
		return instance;
	} else {
		return instance;
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

class fuzzyController{
	private:
		static bool instanceFlag;
		static fuzzyController *instance;
	    Engine* engine;
		InputVariable* backward;
		InputVariable* sideward;
		InputVariable* up;
		InputVariable* rotation;
		OutputVariable* backwardSpeed;
		OutputVariable* sidewardSpeed;
		OutputVariable* upSpeed;
		OutputVariable* rotationSpeed;

	public:
		static fuzzyController* getInstance();
		void init();
		resultSet getFISResult(float, float, float ,float);
		~fuzzyController() {
			instanceFlag = false;
		}
};
bool fuzzyController::instanceFlag = false;
fuzzyController* fuzzyController::instance = NULL;
fuzzyController* fuzzyController::getInstance() {
	if (!instanceFlag) {
		instance = new fuzzyController();
		instanceFlag = true;
		return instance;
	} else {
		return instance;
	}
}

void fuzzyController::init(){
	engine = new Engine;
	engine->setName("input");
	engine->setDescription("");
	backward = new InputVariable;
	backward->setName("backward");
	backward->setDescription("");
	backward->setEnabled(true);
	backward->setRange(-1.000, 1.000);
	backward->setLockValueInRange(false);
	backward->addTerm(new Trapezoid("strongForward", -2.000, -2.000, -0.800, -0.400));
	backward->addTerm(new Trapezoid("mediumForward", -0.900, -0.600, -0.400, 0.000));
	backward->addTerm(new Trapezoid("mediumBackward", 0.000, 0.400, 0.600, 0.900));
	backward->addTerm(new Trapezoid("strongBackward", 0.400, 0.800, 2.000, 2.000));
	backward->addTerm(new Trapezoid("zero", -0.600, -0.200, 0.200, 0.600));
	engine->addInputVariable(backward);

	sideward = new InputVariable;
	sideward->setName("sideward");
	sideward->setDescription("");
	sideward->setEnabled(true);
	sideward->setRange(-1.000, 1.000);
	sideward->setLockValueInRange(false);
	sideward->addTerm(new Trapezoid("strongLeft", -2.000, -2.000, -0.800, -0.400));
	sideward->addTerm(new Trapezoid("mediumLeft", -0.900, -0.600, -0.400, 0.000));
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
	rotation->addTerm(new Trapezoid("strongLeft", -2.000, -2.000, -0.800, -0.400));
	rotation->addTerm(new Trapezoid("mediumLeft", -0.900, -0.600, -0.400, 0.000));
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
	backwardSpeed->addTerm(new Triangle("strongForward", -1.200, -1.000, -0.700));
	backwardSpeed->addTerm(new Trapezoid("mediumForward", -1.000, -0.850, -0.600, -0.250));
	backwardSpeed->addTerm(new Trapezoid("mediumBackward", 0.250, 0.600, 0.850, 1.000));
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
	sidewardSpeed->addTerm(new Trapezoid("mediumLeft", -1.000, -0.850, -0.600, -0.250));
	sidewardSpeed->addTerm(new Trapezoid("mediumRight", 0.250, 0.600, 0.850, 1.000));
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
	upSpeed->addTerm(new Trapezoid("mediumDown", -1.000, -0.850, -0.600, -0.250));
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
	rotationSpeed->addTerm(new Trapezoid("mediumLeft", -1.000, -0.850, -0.600, -0.250));
	rotationSpeed->addTerm(new Trapezoid("mediumRight", 0.250, 0.600, 0.850, 1.000));
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
	ruleBlock->addRule(Rule::parse("if backward is strongForward then backwardSpeed is strongForward", engine));
	ruleBlock->addRule(Rule::parse("if backward is mediumForward then backwardSpeed is mediumForward", engine));
	ruleBlock->addRule(Rule::parse("if backward is mediumBackward then backwardSpeed is strongBackward", engine));
	ruleBlock->addRule(Rule::parse("if backward is strongBackward then backwardSpeed is strongBackward", engine));
	ruleBlock->addRule(Rule::parse("if backward is zero then backwardSpeed is zero", engine));
	ruleBlock->addRule(Rule::parse("if sideward is strongLeft then sidewardSpeed is strongLeft", engine));
	ruleBlock->addRule(Rule::parse("if sideward is mediumLeft then sidewardSpeed is mediumLeft", engine));
	ruleBlock->addRule(Rule::parse("if sideward is mediumRight then sidewardSpeed is mediumRight", engine));
	ruleBlock->addRule(Rule::parse("if sideward is strongRight then sidewardSpeed is strongRight", engine));
	ruleBlock->addRule(Rule::parse("if sideward is zero then sidewardSpeed is zero", engine));
	ruleBlock->addRule(Rule::parse("if backward is mediumBackward then backwardSpeed is strongBackward", engine));
	ruleBlock->addRule(Rule::parse("if up is strongDown then upSpeed is strongDown", engine));
	ruleBlock->addRule(Rule::parse("if up is mediumDown then upSpeed is mediumDown", engine));
	ruleBlock->addRule(Rule::parse("if up is mediumUp then upSpeed is mediumUp", engine));
	ruleBlock->addRule(Rule::parse("if up is strongUp then upSpeed is strongUp", engine));
	ruleBlock->addRule(Rule::parse("if up is zero then upSpeed is zero", engine));
	ruleBlock->addRule(Rule::parse("if rotation is strongLeft then rotationSpeed is strongLeft", engine));
	ruleBlock->addRule(Rule::parse("if rotation is mediumLeft then rotationSpeed is mediumLeft", engine));
	ruleBlock->addRule(Rule::parse("if rotation is mediumRight then rotationSpeed is mediumRight", engine));
	ruleBlock->addRule(Rule::parse("if rotation is strongRight then rotationSpeed is strongRight", engine));
	ruleBlock->addRule(Rule::parse("if rotation is zero then rotationSpeed is zero", engine));
	ruleBlock->addRule(Rule::parse("if backward is strongForward and sideward is strongLeft then backwardSpeed is strongForward and sidewardSpeed is strongLeft and rotationSpeed is strongLeft", engine));
	ruleBlock->addRule(Rule::parse("if backward is mediumForward and sideward is mediumLeft then backwardSpeed is mediumForward and sidewardSpeed is mediumLeft and rotationSpeed is mediumLeft", engine));
	ruleBlock->addRule(Rule::parse("if backward is strongForward and sideward is strongRight then backwardSpeed is strongForward and sidewardSpeed is strongRight and rotationSpeed is strongRight", engine));
	ruleBlock->addRule(Rule::parse("if backward is mediumForward and sideward is mediumRight then backwardSpeed is mediumForward and sidewardSpeed is mediumRight and rotationSpeed is mediumRight", engine));
	ruleBlock->addRule(Rule::parse("if backward is mediumForward and sideward is strongRight then backwardSpeed is mediumForward and sidewardSpeed is strongRight and rotationSpeed is strongRight", engine));
	ruleBlock->addRule(Rule::parse("if backward is mediumForward and sideward is strongLeft then backwardSpeed is mediumForward and sidewardSpeed is strongLeft and rotationSpeed is strongLeft", engine));
	ruleBlock->addRule(Rule::parse("if backward is mediumForward and sideward is zero then backwardSpeed is mediumForward and sidewardSpeed is zero and rotationSpeed is zero", engine));
	ruleBlock->addRule(Rule::parse("if backward is strongForward and sideward is zero then backwardSpeed is strongForward and sidewardSpeed is zero and rotationSpeed is zero", engine));
	ruleBlock->addRule(Rule::parse("if backward is strongForward and sideward is mediumLeft then backwardSpeed is strongForward and sidewardSpeed is mediumLeft and rotationSpeed is strongLeft", engine));
	ruleBlock->addRule(Rule::parse("if backward is strongForward and sideward is mediumRight then backwardSpeed is strongForward and sidewardSpeed is mediumRight and rotationSpeed is strongRight", engine));
	ruleBlock->addRule(Rule::parse("if backward is mediumBackward then backwardSpeed is mediumBackward", engine));
	engine->addRuleBlock(ruleBlock);
}

resultSet fuzzyController::getFISResult(float back, float side, float upValue, float rotateRight){
    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);
    backward->setValue(back);
    sideward->setValue(side);
    up->setValue(upValue);
    rotation->setValue(rotateRight);
    engine->process();

    resultSet result;
    result.backwardSpeed = backwardSpeed->getValue();
    result.sidewardSpeed = sidewardSpeed->getValue();
    result.upSpeed = upSpeed->getValue();
    result.rotationSpeed = rotationSpeed->getValue();

  return result;
}

void messageCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
	SkeletonPoints* points = SkeletonPoints::getInstance();
	fuzzyController* fc = fuzzyController::getInstance();
	float up,rotateRight,back,side;
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

	//calculate joint diffs
	//might need to smooth Signal
	up = points->getJoint(jType.leftHand).y - points->getJoint(jType.leftShoulder).y + points->getJoint(jType.rightHand).y - points->getJoint(jType.rightShoulder).y;
	side = points->getJoint(jType.leftHand).y - points->getJoint(jType.rightHand).y;
	rotateRight = points->getJoint(jType.rightHand).z  -points->getJoint(jType.leftHand).z;
	back = points->getJoint(jType.neck).z - points->getJoint(jType.torso).z;
	resultSet resultSpeeds = fc->getFISResult(back,side,up,rotateRight);
	ROS_INFO("backSpeed: %d", resultSpeeds.backwardSpeed);
}

int main(int argc, char **argv) {

   ros::init(argc, argv, "listener");
   ros::NodeHandle n;
   fuzzyController::getInstance()->init();
   ros::Subscriber sub = n.subscribe("/tf", 1000, messageCallback);

   ros::spin();

	return 0;
}

