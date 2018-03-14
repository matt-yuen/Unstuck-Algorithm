#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <cmath>
#include <std_msgs/String.h>
using namespace std;

// #include <stdio.h>
// #include <time.h>
// #include <stdlib.h>

ros::Subscriber sub;
ros::Publisher pub, statePub;
nav_msgs::Odometry prevOdom;
nav_msgs::Odometry originalStuckOdom;

// deque lastTwentySec;
double initialYaw;
int cnt = 0;
int state, subState;

// Changed
double roll, pitch, yaw;

// Added
double backDistance;

void turtleCallback(const nav_msgs::Odometry&msg);
void extractRPY(double & r, double & p, double& y, const nav_msgs::Odometry& msg); // converts the odometry message to roll pitch and yaw
// void extractPosition()
void sampleTurn (double angle, double yaw, double initialYaw);

//base functions
bool checkIfStuck();
void move(double linSpeed, double angularSpeed);
bool atDestination(double distance, const nav_msgs::Odometry& currPos, double yaw); //maybe instead take in the aldready processed x and y coordinates
bool turned(double targetAngle, double initYaw, double yaw); //all angles in rad
void updatePrevOdom(const nav_msgs::Odometry& msg);
void updateStateMsg(string message); // to publish a message to the statePub channel
bool algPartOne (const nav_msgs::Odometry & msg, double yaw);

//Variables used for testing
int turnCount;

int main(int a, char ** b)
{
    ros::init(a, b, "BobTheTurtle");

    ros::NodeHandle n;
    cnt = 0;
    turnCount = 0;

    pub  = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    statePub = n.advertise<std_msgs::String>("/unstuck_node/message",1);

    sub = n.subscribe("/odom", 1, turtleCallback);
    //state = subState = 0;

    //Testing
	state = 1;
    subState = 0;

    ros::Rate loop_rate(60);
    while( ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
  

void turtleCallback (const nav_msgs::Odometry&msg)
{
    //get all relevant odometry info:
    int x = msg.pose.pose.position.x;
    int y = msg.pose.pose.position.y;
    extractRPY(roll,pitch,yaw, msg);

    if(cnt == 0)
        initialYaw = yaw;
    cnt++;

/*
    if(checkIfStuck() && state == 0)
    {
        state = 1;
        subState = 0;
    }else
    {
        updateStateMsg("unstuck");
    }

    if(state != 0)
    {
        updateStateMsg("stuck");
    }
*/

    //Don't need this function anymore because we have turned
    //sampleTurn(M_PI/2, yaw, initialYaw);

    //Testing "turned" function
/*    if (!turned(M_PI/2, initialYaw, yaw))
    {
    	ROS_INFO("still turning");
    	move(0, 0.1);
    }
    else
    {
    	ROS_INFO("done turning");
    	move(0, 0);	
    }*/

    //Testing atDestination function
    /*if (!atDestination(2, msg, yaw) && turnCount == 0)
    {
    	ROS_INFO("still moving");
    	move(0.2, 0);
    }
    else
    {
    	turnCount++;
    	ROS_INFO("done moving");
    	move(0,0);
    }*/

    //Testing alg part one
    if (state == 1)
    	algPartOne(msg, yaw);
}

void extractRPY(double & r, double & p, double& y, const nav_msgs::Odometry& msg)
{
    double x1 = msg.pose.pose.orientation.x;
    double y1 = msg.pose.pose.orientation.y;
    double z1 = msg.pose.pose.orientation.z;
    double w1 = msg.pose.pose.orientation.w;
    tf::Quaternion q(x1,y1,z1,w1);
    tf::Matrix3x3 mat(q);
    mat.getRPY(r,p,y);

}

void extractPosition(double & x, double & y , double & z, const nav_msgs::Odometry&msg)
{


}

void sampleTurn (double angle, double yaw, double initialYaw)
{
    double  ang = 0.2;
    if(fabs( fabs(yaw-initialYaw) - angle) < 1e-1)
    {
        
        ang = 0;
        ROS_INFO("Reached!");
    }
    move(0,ang);
    // return nextTwist;
}

void move(double linSpeed, double angularSpeed)
{
    geometry_msgs::Twist nextTwist;
    nextTwist.linear.x= linSpeed;
    nextTwist.linear.y= 0;
    nextTwist.linear.z= 0;

    nextTwist.angular.z = angularSpeed;
    nextTwist.angular.x = 0;
    nextTwist.angular.y = 0;

    pub.publish(nextTwist);
}

void updatePrevOdom(const nav_msgs::Odometry & currOdom)
{
    prevOdom = currOdom;
}

void updateStateMsg(string msg)
{
    
    std_msgs::String s;
    s.data = msg;
    statePub.publish(s);
    // ROS_INFO("%s",s.data.c_str());

}

bool checkIfStuck()
{
    return 0;
}

bool atDestination(double distance, const nav_msgs::Odometry& currPos, double yaw)
{
    double xCurrent = currPos.pose.pose.position.x;
    double yCurrent = currPos.pose.pose.position.y;
    double xPrev = prevOdom.pose.pose.position.x;
    double yPrev = prevOdom.pose.pose.position.y;
    double xDest = xPrev + distance*cos(yaw);
    double yDest = yPrev + distance*sin(yaw);

    if (fabs(xDest - xCurrent) < 1e-1 && fabs(yDest - yCurrent) < 1e-1)
    {
    	updatePrevOdom(currPos);
        return true;
    }
    
    return false;
}

bool turned(double targetAngle, double initYaw, double yaw)
{
	double targetYaw = initialYaw + targetAngle; //Angle from positive x-axis after turning the desired "targetAngle"

	if (fabs(targetYaw - yaw) < 1e-2)
		return true;

	return false;
}

bool algPartOne (const nav_msgs::Odometry & msg, double yaw)
{
	//****these don't work...->??
	int linearSpeed = 0;
	int angularSpeed = 0;

	switch (subState)
	{
		//******RESET ALL CHECKING VARIABLES IN EACH SUBCASE?********

		//Move back 5 m
		case 0:
			if (!atDestination(2, msg, yaw))
			{
				ROS_INFO("Moving");
				move(0.2, 0);
			}
			else
			{
				ROS_INFO("Done moving");
				subState = 1;
				move(0, 0);
				updatePrevOdom(msg);
				initialYaw = yaw;
				extractRPY(roll, pitch, yaw, msg);
			}

			//If can't move back 5 m successfully, change state to 2 
			if (checkIfStuck())
			{
				state = 2;
				subState = 0;
				updatePrevOdom(msg);
				initialYaw = yaw;
				extractRPY(roll, pitch, yaw, msg);
				ROS_INFO("Stuck");
			}

			break;
		//Turn right 45, assume successful
		case 1:
			if (!turned(-M_PI/4, initialYaw, yaw))// && !turnedYet) 
			{
				ROS_INFO("TURNING");
				move(0, -0.3);
			}
			else
			{
				ROS_INFO("DONE TURNING");
				subState = 2;
				move(0, 0);
				updatePrevOdom(msg);
				initialYaw = yaw;
				extractRPY(roll, pitch, yaw, msg);
			}

			break;
		//Go forward 
		case 2:
			if (!atDestination(5/cos(M_PI/4), msg, yaw))
			{
				ROS_INFO("Moving");
				move(0.2, 0);
			}
			else //*****UNSTUCK*****
			{
				ROS_INFO("Done moving");
				//subState = 4;
				move(0, 0);
				updatePrevOdom(msg);
				initialYaw = yaw;
				extractRPY(roll, pitch, yaw, msg);
			}

			if (checkIfStuck())
			{
				subState = 3;
				updatePrevOdom(msg);
				ROS_INFO("Stuck");
				initialYaw - yaw;
				extractRPY(roll, pitch, yaw, msg);
				move(0, 0);
				backDistance = fabs(msg.pose.pose.orientation.x - prevOdom.pose.pose.orientation.x) / cos(yaw);
			}

			break;
		//Stuck from case 2, go back to prevOdom
		case 3:
			if (!atDestination(-backDistance, msg, yaw))
			{
				ROS_INFO("Moving");
				move(-0.2, 0);
			}
			else
			{
				ROS_INFO("Done moving");
				subState = 4;
				move(0, 0);
				updatePrevOdom(msg);
				initialYaw = yaw;
				extractRPY(roll, pitch, yaw, msg);
			}

			break;
		case 4:
			if (!turned(M_PI/2, initialYaw, yaw))
			{
				ROS_INFO("Moving");
				move(0, 0.3);
			}
			else
			{
				ROS_INFO("Done moving");
				subState = 5;
				move(0, 0);
				updatePrevOdom(msg);
				initialYaw = yaw;
				extractRPY(roll, pitch, yaw, msg);
			}

			break;
		case 5:
			if (!atDestination(5/cos(M_PI/4), msg, yaw))
			{
				ROS_INFO("Moving");
				move(0.2, 0);
			}
			else //*****UNSTUCK*****
			{
				ROS_INFO("Done moving");
				//subState = 4;
				move(0, 0);
				updatePrevOdom(msg);
				initialYaw = yaw;
				extractRPY(roll, pitch, yaw, msg);
			}

			if (checkIfStuck())
			{
				subState = 6;
				updatePrevOdom(msg);
				ROS_INFO("Stuck");
				initialYaw - yaw;
				extractRPY(roll, pitch, yaw, msg);
				move(0, 0);
				backDistance = fabs(msg.pose.pose.orientation.x - prevOdom.pose.pose.orientation.x) / cos(yaw);
			}

			break;
		case 6:
			if (!atDestination(-backDistance, msg, yaw))
			{
				ROS_INFO("Moving");
				move(-0.2, 0);
			}
			else
			{
				ROS_INFO("Done moving");
				subState = 4;
				move(0, 0);
				updatePrevOdom(msg);
				initialYaw = yaw;
				extractRPY(roll, pitch, yaw, msg);
			}

			break;
	}
}