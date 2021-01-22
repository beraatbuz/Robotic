#include "ros/ros.h"
#include <ros/console.h>
#include <string> 
#include <robot_control/RobotVision.h>
#include <geometry_msgs/Twist.h>
#include <robot_control/BallPose.h>
#include <iostream> 
#include <utility> 
#include <cmath>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose2D.h>

#define PI_Local 3.14159265



int ballAngle;
int goalAngle;

ros::Publisher pub;

bool actionState = false;
bool odomSemaphore = true;
int missionPhase = 1;
bool blindMode = false;

int  ballImageX;
int  ballImageY;

float ballX = 0;
float ballY = 0;
float ballStatus = false;

std::pair <int, int> imageInfo((320/2),(240/2));

geometry_msgs::Twist navigation;
bool navigationStatus = false;

ros::ServiceClient client;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float robotX;
float robotY;
float robotTheta;

float camera_base = 0.125;

float aimX = -3.0;
float aimY = 0.0;
float goal_buff = 0.9;
float threshold = 0.05;

float ballDistance;
bool ballCamStatus = false;


float saturateVelocity(float velocity, bool angular){

  int sign = 1;
  if(velocity < 0) sign = -1;
  
  velocity = std::abs(velocity);

  if(angular == true){
  	 if(velocity > 0.5){
	    velocity = 0.5;
	  }
	  if(velocity < 0.02) velocity = 0.02;
	  
  }
  else{
  	if(velocity > 0.5){
	    velocity = 0.5;
	  }
	  if(velocity < 0.02) velocity = 0.02;
  }
  return sign*velocity;

}

move_base_msgs::MoveBaseGoal action_handler(geometry_msgs::Pose2D aimlPose){
  move_base_msgs::MoveBaseGoal aim;

  aim.target_pose.header.frame_id = "odom";
  aim.target_pose.header.stamp = ros::Time::now();

  aim.target_pose.pose.position.x = aimlPose.x;
  aim.target_pose.pose.position.y = aimlPose.y;

  tf2::Quaternion myQuaternion;
  
  myQuaternion.setRPY( 0, 0, aimlPose.theta);
  myQuaternion.normalize();

  aim.target_pose.pose.orientation = tf2::toMsg(myQuaternion);

  return aim;
}

void odom_call_back(const nav_msgs::Odometry::ConstPtr& msg){

    
	robotX = msg->pose.pose.position.x;
	robotY = msg->pose.pose.position.y;
	tf2::Quaternion quat;
	tf2::fromMsg(msg->pose.pose.orientation, quat);
	double roll, pitch, yaw;
	tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	robotTheta = yaw;
    

    float x_sq = pow((robotX - aimX),2);
    float y_sq = pow((robotY - aimY),2);
    float robot_goal_dist = std::sqrt( x_sq + y_sq);

    float r_x = cos(robotTheta);
    float r_y = sin(robotTheta);

    float rg_x = (aimX  - robotX)/robot_goal_dist;
    float rg_y = (aimY - robotY)/ robot_goal_dist;
    float align_theta = asin(r_x*rg_y - r_y*rg_x); 

    if(missionPhase == 5){

      blindMode = true;
      if(robot_goal_dist > goal_buff){
          geometry_msgs::Twist robot_cmd_vel;

          int xerror = ( (imageInfo.first) - ballImageX);

          robot_cmd_vel.linear.x = 0.2;
            
          robot_cmd_vel.angular.z  = saturateVelocity(align_theta,false);
          robot_cmd_vel.linear.y  = saturateVelocity(float(xerror)/(imageInfo.first), true);
          pub.publish(robot_cmd_vel);

        }
        else{
          missionPhase = 6;
        }
    }
    if(missionPhase == 6){
          blindMode = true;
          geometry_msgs::Twist robot_cmd_vel;

          int xerror = ( (imageInfo.first) - ballImageX);

          robot_cmd_vel.linear.x = 0.5;
          robot_cmd_vel.angular.z  = saturateVelocity(align_theta,false);
          robot_cmd_vel.linear.y  = saturateVelocity(float(xerror)/(imageInfo.first), true);
          pub.publish(robot_cmd_vel);

          ROS_DEBUG_STREAM(robot_cmd_vel.linear.x);

          robot_cmd_vel.linear.x = 0;
          pub.publish(robot_cmd_vel);
	  ros::Duration(1).sleep();
          missionPhase = 10;
	  aimX = 0.0;
	  aimY = 0.0;
    }
    if(missionPhase == 10){

        geometry_msgs::Twist robot_cmd_vel;
        if((robotX == aimX && robotY == aimY) || robot_goal_dist <0.05){

            robot_cmd_vel.linear.x = 0;
            pub.publish(robot_cmd_vel);
            missionPhase = 50;
            ros::shutdown();
        }
        if(robotX == aimX && robotY == aimY){
            missionPhase = 50;
            robot_cmd_vel.linear.x = 0;
            pub.publish(robot_cmd_vel);
        }
	float angle_to_goal = atan2(aimY-robotY,aimX-robotX);
	

	    if (abs(angle_to_goal - robotTheta) > 0.1){
		robot_cmd_vel.linear.x = 0.0;

		robot_cmd_vel.angular.z = 0.3;
}
	    else{
		robot_cmd_vel.linear.x = 0.5;
		robot_cmd_vel.angular.z = 0.0;
		}
        pub.publish(robot_cmd_vel);
	
	
    }


  }

void call_back_vision(const robot_control::RobotVision::ConstPtr& msg){

  ballDistance =  msg->DistBall;
  ballCamStatus = msg->Ball;
  ballImageX = msg->BallCenterX;
  ballImageY = msg->BallCenterY;

  if(blindMode == true) return;

  if(ballCamStatus == false) missionPhase =1;
  

  if(actionState == false){
    if(missionPhase == 1){
        if (ballCamStatus  == false){ 
          geometry_msgs::Twist robot_cmd_vel;
          robot_cmd_vel.angular.z  = 0.5;
          pub.publish(robot_cmd_vel);
        }else{
          missionPhase = 2; 
        }
    }
    else if(missionPhase == 2){ 

        geometry_msgs::Twist robot_cmd_vel;

        int xerror = ( (imageInfo.first) - msg->BallCenterX);

        if(std::abs(xerror) < threshold*(imageInfo.first)){
            robot_cmd_vel.angular.z  = 0;
            pub.publish(robot_cmd_vel);
            missionPhase = 3;

        }
        else{ 
            float velocity = float(xerror)/(imageInfo.first);
            robot_cmd_vel.angular.z = saturateVelocity(velocity,false);
            pub.publish(robot_cmd_vel);
          }
      
     }
     else if (missionPhase == 3){
        robot_control::BallPose srv;
        srv.request.dist = ballDistance;
        if (client.call(srv))
        {
          ballX = srv.response.x;
          ballY = srv.response.y;
          ballStatus = true;
          missionPhase = 4;
        }
      else
        {
          ROS_ERROR("Ball Pose failed");
        }
     }
     else if(missionPhase == 4){
        actionState = true;
      }
    }
}



geometry_msgs::Pose2D behind_the_ball(){

  geometry_msgs::Pose2D out;

  out.theta = atan2(aimY-ballY,aimX - ballX);

  
  if(ballStatus){
      out.x = ballX - cos(out.theta)*.40;
      out.y = ballY - sin(out.theta)*.40;
  }

  return out;

  }


int main(int argc, char **argv)
{
  if(*argv[1]=='1'){
      aimX=0.0;
      aimY=3.0;
  }
 else if(*argv[1]=='2'){
      aimX=-3.0;
      aimY=0.0;
  }
 else if(*argv[1]=='3'){
      aimX=0.0;
      aimY=-3.0;
  }
  ros::init(argc, argv, "central_server");
  ros::NodeHandle n("~");

  ros::Subscriber sub_vision = n.subscribe("/robot_vision", 10, call_back_vision);
  ros::Subscriber sub_odom = n.subscribe("/odom", 100, odom_call_back);

  client = n.serviceClient<robot_control::BallPose>("/ball_pose_srv");


  pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  MoveBaseClient action_client("move_base", true);

  while(!action_client.waitForServer(ros::Duration(5.0))){
    ROS_INFO("move_base action");
  }

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    if(actionState == true){

      if(missionPhase == 4){

        geometry_msgs::Pose2D  aimlPose =  behind_the_ball();
        move_base_msgs::MoveBaseGoal aim = action_handler(aimlPose);

        action_client.sendGoal(aim);
        action_client.waitForResult();

        if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
          missionPhase = 5;
        }
        else{
          ROS_INFO("The base failed to move forward");
        }

        actionState = false;  
      }

    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
