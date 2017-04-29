#include "arpa/inet.h"
#include "netinet/in.h"
#include "stdio.h"
#include "sys/types.h"
#include "sys/socket.h"
#include "unistd.h"
#include "string.h"
#include "stdlib.h"
#include "signal.h"
#include "unistd.h"
#include "fcntl.h"
#include <stdint.h>
#include <inttypes.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include "math.h"
#include <ros/time.h>

// UDP buffer length
#define BUFLEN 512
// UDP port to receive from
#define PORT 2014                                            

// Asynchronous UDP communication
#define ASYNC

// UDP port to send data to
#define PORT_BRD 2013                                        
// Tiva "Knee" encoder board IP
#define BRD_IP "192.168.1.11"                                




// Global variables
bool gotMsg = false; // Flag set high when message is received from UDP
int sock; // The socket identifier for UDP Rx communication
int msgs = 0; // Incoming message counter
struct sockaddr_in si_pwm;  // Struct for UDP send data socket
ssize_t SendPWMBytes = 2;  // Number of bytes to send for PWM command
char SendBuffer[6];  // UDP Send Buffer
int broad;	// The socket identifier for UDP Tx communication
int slen=sizeof(si_pwm);  // Size of sockaddr_in strut
float pi = 4.0*atan(1.0);

int32_t encoderPos = 0;  // Place the received encoder value here
float angle, frc, prev_frc, compression ; 
float l1 = 0.05 ;    // knee top link length
float l2 = 0.065 ;    // knee bot link length
float nl = 0.089 ;    // initial length
float linit = sqrt(l1*l1 + l2*l2 - 2.0*l1*l2*cos(360.0*361/1999.0*pi/180.0) - 0.01*0.01) ; 
float k = 5000.0 ;  // stiffness

float norm_compression ;

// Generic error function
void error(char *s)
{
    perror(s);
    exit(1);
}

// Signal handler for asynchronous UDP
void sigio_handler(int sig)
{
   char buffer[BUFLEN]="";
   unsigned char val[4];
   struct sockaddr_in si_other;
   unsigned int slen=sizeof(si_other);
   ssize_t rcvbytes = 0;

   // Receive available bytes from UDP socket
   if ((rcvbytes = recvfrom(sock, &buffer, BUFLEN, 0, (struct sockaddr *)&si_other, &slen))==-1)
       error("recvfrom()");
   else
   {
	   // Parse data , 1 int32 value
     if(buffer[0] == 0x42)
     {
           //ROS_INFO(" received");
	   val[3] = (unsigned char)buffer[4];
	   val[2] = (unsigned char)buffer[3];
	   val[1] = (unsigned char)buffer[2];
	   val[0] = (unsigned char)buffer[1];
	   memcpy(&encoderPos, &val, 4);
           frc = k*(sqrt(l1*l1 + l2*l2 - 2.0*l1*l2*cos(360.0*encoderPos/1999.0*pi/180.0) - 0.01*0.01)- linit) ;
	  // ROS_INFO("%f  force-------------------------------------------", frc);
       // Raise flag that we received a message
       gotMsg = true;
     }
   }
}

// Function to enable asynchronous UDP communication
int enable_asynch(int sock)
{
  int stat = -1;
  int flags;
  struct sigaction sa;

  flags = fcntl(sock, F_GETFL);
  fcntl(sock, F_SETFL, flags | O_ASYNC); 

  sa.sa_flags = 0;
  sa.sa_handler = sigio_handler;
  sigemptyset(&sa.sa_mask);

  if (sigaction(SIGIO, &sa, NULL))
    error("Error:");

  if (fcntl(sock, F_SETOWN, getpid()) < 0)
    error("Error:");

  if (fcntl(sock, F_SETSIG, SIGIO) < 0)
    error("Error:");

  return 0;
}


// Main Function
int main(int argc, char **argv)
{
  struct sockaddr_in si_me, si_other;
  int i, slen=sizeof(si_other), msg_count;
  char buf[BUFLEN], strout[28];
  
  msg_count = 0;
  memset(SendBuffer, 0, 6);
  
  // Initialize UDP socket for data transmission
  if ((broad=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
       error("socket");
	   
  memset((char *) &si_pwm, 0, sizeof(si_pwm));
  si_pwm.sin_family = AF_INET;
  si_pwm.sin_port = htons(PORT_BRD);
   
  if (inet_aton(BRD_IP, &si_pwm.sin_addr)==0) {
       error("inet_aton() failed\n");
       exit(1);
  }
  
  SendBuffer[0] = 0x31;

  // Initialize ROS node
  ros::init(argc, argv, "VV_knee_interface");
  ros::NodeHandle n;
  // Initialize the publisher for Force data post
  ros::Publisher force_pub = n.advertise<std_msgs::Float64>("/reaction_force", 1000);
  // Initialize the publisher for compression data post
  ros::Publisher compression_pub = n.advertise<std_msgs::Float64>("/compression", 1000);
  // // Initialize the publisher for phase state post
  // ros::Publisher phase_pub = n.advertise<std_msgs::Int32>("/phase_state", 1000);

  ros::Rate loop_rate(1000); 

  // Wait for ROS node to initialize
  while (!ros::ok());
  
  // Initialize UDP socket for data reception
  if ((sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
     error("socket");

  memset((char *) &si_me, 0, sizeof(si_me));
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(PORT);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(sock, (struct sockaddr *)&si_me, sizeof(si_me))==-1)
      error("bind");

  enable_asynch(sock);
  
  ROS_INFO("Starting communication with TiVa board.");
  ROS_INFO("Communication with TiVa board established.");
  ROS_INFO("Reading Reaction Forces");

  std_msgs::Float64 force_msg;
  std_msgs::Float64 compression_msg;
  // std_msgs::Int32 phase_msg;
  prev_frc = 0.0; 

  while (ros::ok())
  {
    if(gotMsg)
    {
      msg_count++;

      force_msg.data = frc ;
      force_pub.publish(force_msg);

      compression_msg.data = frc/k ; 
      compression_pub.publish(compression_msg);

      norm_compression = frc/k/nl ; 
    
      // if (norm_compression < 0.02)
      // {
		    // //ROS_INFO("FLIGHT FASE ************************************************");
		    // phase_msg.data = 1 ; 	
      // }
      // else
      // {
		    // //ROS_INFO("%f  Contact force -------------------------------------- ",force);
		    // phase_msg.data = 0 ;
      // }

      // phase_pub.publish(phase_msg);

      if(msg_count >= 2000)
      {
		    msg_count = 0;
		    //ROS_INFO("%f  force", frc);
      }
      gotMsg = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
