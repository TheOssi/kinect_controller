
//struct for 3D Points
struct point {
	float x;
	float y;
	float z;
};
//Struct with Indices for Joints
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
//Class storing the Joints
class SkeletonPoints {
private:
	point points[14];

public:
	void setJoint(int, float, float, float);
	point getJoint(int);

};

