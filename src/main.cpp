#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf2_msgs/TFMessage.h>
#include <string>

using namespace std;

struct point{
 float x;
 float y;
 float z;
};



class SkeletonPoints
{
private:
    static bool instanceFlag;
    static SkeletonPoints *single;
    point rightHand;
    SkeletonPoints()
    {
        //private constructor
    }
public:
    static SkeletonPoints* getInstance();
    point getRightHand();
    void setRightHand(float,float,float);

    ~SkeletonPoints()
    {
        instanceFlag = false;
    }
};

bool SkeletonPoints::instanceFlag = false;
SkeletonPoints* SkeletonPoints::single = NULL;
SkeletonPoints* SkeletonPoints::getInstance()
{
    if(! instanceFlag)
    {
        single = new SkeletonPoints();
        instanceFlag = true;
        return single;
    }
    else
    {
        return single;
    }
}




point SkeletonPoints::getRightHand(){
  return rightHand;
}

void SkeletonPoints::setRightHand(float x, float y,float z){
  rightHand.x = x;
  rightHand.y = y;
  rightHand.z = z;
}

void messageCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
  SkeletonPoints* points;
  points = SkeletonPoints::getInstance();
  float x = msg->transforms[0].transform.translation.x;
  float y =msg->transforms[0].transform.translation.y;
  float z = msg->transforms[0].transform.translation.z;
  string childframe(msg->transforms[0].child_frame_id.c_str());
  if(childframe == "right_hand_1"){
	  points->setRightHand(x,y,z);
	  point rHand = points->getRightHand();
	  ROS_INFO("%6.4lf | %6.4lf | %6.4lf", rHand.x, rHand.y,rHand.z);
  }


}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "listener");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/tf", 1000, messageCallback);

	ros::spin();

  return 0;
}


