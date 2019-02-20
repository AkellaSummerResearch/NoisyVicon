#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <stdlib.h>
#include "vicon_driver/vicon_driver.h"
#include "vicon_driver/vicon_calib.h"
#include "vicon/Subject.h"
#include "vicon/Markers.h"
#include "vicon/SetPose.h"
#include <math.h>
#include <time.h>
#define pi 3.14159265

geometry_msgs::PoseWithCovarianceStamped poseCov;
ros::Subscriber vicon_sub;
double roll,pitch,yaw,x,y,z;
ros::Publisher noise_pub;
ros::Time prevWrite;
ros::Duration writeDelay(0.1);

void viconCallback(vicon::Subject bilbo)
{
  float bilboz = bilbo.position.z;
  // yaw = atan2(sqrt(bilbo.position.x*bilbo.position.x+bilbo.position.y*bilbo.position.y),bilboz)*180/pi;
  // roll = atan2(sqrt(bilboz*bilboz+bilbo.position.x*bilbo.position.x),bilbo.position.y)*180/pi;
  // pitch = atan2(sqrt(bilboz*bilboz+bilbo.position.y*bilbo.position.y),bilbo.position.x)*180/pi;
  // yaw = atan2(bilbo.position.y,bilbo.position.x)*180/pi;
  // roll = 0;
  // pitch = atan2(bilbo.position.z,sqrt(bilbo.position.x*bilbo.position.x+bilbo.position.y*bilbo.position.y))*180/pi;
  // pose.orientation = tf::createQuaternionMsgFromRollPitchYaw((roll*pi/180),(pitch*pi/180),(yaw*pi/180));
  // float sph = 0.5/(sqrt(bilbo.position.x*bilbo.position.x + bilbo.position.y*bilbo.position.y + bilboz*bilboz));
  // pose.position.x = sph*bilbo.position.x;
  // pose.position.y = sph*bilbo.position.y;
  // pose.position.z = sph*bilboz;
  // if(ros::Time::now()-prevWrite>writeDelay)
  // {
  // arm_pub.publish(pose);
  // prevWrite=ros::Time::now();
  // std::cout<<pose.position.x<<" "<<pose.position.y<<" "<<pose.position.z<<std::endl;
  // }
  std::cout<<(float)(rand()%10 - 5)/10<<std::endl;
  poseCov.pose.pose.position.x = bilbo.position.x + (float)(rand()%10 - 5)/10;
  poseCov.pose.pose.position.y = bilbo.position.y + (float)(rand()%10 - 5)/10;
  poseCov.pose.pose.position.z = bilbo.position.z + (float)(rand()%10 - 5)/10;
  //poseCov.pose.pose.orientation.x = bilbo.position.x + (rand()%5 - 5)/10;
  tf::Quaternion q(bilbo.orientation.x,bilbo.orientation.y,bilbo.orientation.z,bilbo.orientation.w);
  tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
  roll += (rand()%5 - 5);
  pitch += (rand()%5 - 5);
  yaw += (rand()%5 - 5);
  poseCov.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw((roll*pi/180),(pitch*pi/180),(yaw*pi/180));
  poseCov.header = bilbo.header;
  noise_pub.publish(poseCov);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "vicon_noise");
  ros::NodeHandle nh;
  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;
  prevWrite=ros::Time::now();
  vicon_sub = nh.subscribe("/vicon/crazyflie2", 10, viconCallback);
  noise_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/Bilbo/vision_pose/pose_cov",100);
  srand(time(NULL));
  ros::spin();
}