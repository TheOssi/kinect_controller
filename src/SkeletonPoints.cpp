#include "SkeletonPoints.h"

void SkeletonPoints::setJoint(int joint, float x, float y, float z) {
	points[joint].x = x;
	points[joint].y = y;
	points[joint].z = z;
}
point SkeletonPoints::getJoint(int joint) {
	return points[joint];
}
