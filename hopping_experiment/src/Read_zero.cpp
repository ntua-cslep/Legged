#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <inttypes.h>
#include "ros/ros.h"
#include <iostream>
#include <string>
#include <sstream>


#include "std_msgs/Float64.h"
#include <dynamic_reconfigure/server.h>
#include <hopping_experiment/ParamConfig.h>

float rpos, prevpos ; 
float dt  ; 
int steps, ctr ; 

using namespace std;
/*

	ROS node that reads the value of the desired position 
	and publishes it in the desired_position topic.
	
*/

void callback(legged_robot::ParamConfig &config, uint32_t level) {
	rpos =  config.setpoint;
	dt = config.dt;
	steps = config.steps;
}

// Global variables
std_msgs::Float64 setpoint_msg;
float pi = 4.0*atan(1.0);

int main(int argc, char **argv)
{
  prevpos = 0.0 ; 
  // Initialize ROS node
  ros::init(argc, argv, "Read_setpoint");
  ros::NodeHandle nod;
  // Publish for desired position message
  ros::Publisher read_setpoint_pub = nod.advertise<std_msgs::Float64>("/setpoint", 1000);

  dynamic_reconfigure::Server<legged_robot::ParamConfig> Read_setpoint;
  dynamic_reconfigure::Server<legged_robot::ParamConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  Read_setpoint.setCallback(f);

  ros::Rate loop_rate(10000);
  setpoint_msg.data = 0.0;
  ROS_INFO("Reading Setpoint.");

  while (ros::ok())
  {
		if (rpos != prevpos)
		{ 
/*
                         for(ctr=1;ctr<=steps; ctr++)
				{
				setpoint_msg.data =  rpos/steps*ctr*pi/180.0;
				read_setpoint_pub.publish(setpoint_msg);
				ros::Duration(dt).sleep(); 
				}
*/
			setpoint_msg.data =  rpos ;
			read_setpoint_pub.publish(setpoint_msg);
			prevpos = rpos ; 
		}
		else 
		{
			setpoint_msg.data =  rpos ;
			read_setpoint_pub.publish(setpoint_msg);
		}
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
