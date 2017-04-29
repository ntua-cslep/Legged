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

// UDP buffer length
#define BUFLEN 512
// UDP port to receive from
#define PORT 2015                                             


// Asynchronous UDP communication
#define ASYNC

// UDP port to send data to
#define PORT_BRD 2016                                         
// Tiva Back Left Leg board IP
#define BRD_IP "192.168.1.12"                                

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sstream>


// Global variables
bool gotMsg = false; // Flag set high when message is received from UDP
int sock; // The socket identifier for UDP Rx communication
int32_t encoderPos = 0;  // Place the received encoder value here
int32_t pos = 0;  // Place the actual value here
int32_t prevpos = 0;  // previous actual position
int32_t prevencoderpos = 0;  //previous encoder position
int msgs = 0; // Incoming message counter
int msgss = 0; // Incoming message counter
struct sockaddr_in si_pwm;  // Struct for UDP send data socket
ssize_t SendPWMBytes = 2;  // Number of bytes to send for PWM command
char SendBuffer[6];  // UDP Send Buffer
int broad;  // The socket identifier for UDP Tx communication
int slen=sizeof(si_pwm);  // Size of sockaddr_in strut
float pi = 4.0*atan(1.0);


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
           prevencoderpos = encoderPos ;
     memcpy(&encoderPos, &val, 4);
           if (((prevencoderpos < 50000) && (encoderPos > 50000)) || ((pos < 0) && (encoderPos < prevencoderpos))) 
           {
                prevpos = pos ;
                pos = encoderPos - 104000 ; 
           }
           else 
           {
        prevpos = pos ;  
        pos = encoderPos ; 
           }
           if (encoderPos = 0) 
               pos = encoderPos ; 
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

// Callback function for reception of PWM message from topic
void PWMCallback(const std_msgs::Float64::ConstPtr& msg)
{
  // Extract the duty cycle value and send it to the Tiva board via UDP
  SendBuffer[1] = (int8_t) msg->data;
  if (sendto(broad, SendBuffer, SendPWMBytes, 0, (struct sockaddr *)&si_pwm, slen)==-1)
    error("sendto()");
  // Print-out for debugging
  msgs++;
  if(msgs == 10)
  {
    msgs = 0;
    //ROS_INFO("I heard: [%f]", msg->data);
  }
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
  ros::init(argc, argv, "Main_interface");
  ros::NodeHandle n;
  // Initialize the publisher for Encoder data post
  ros::Publisher position_interface_pub = n.advertise<std_msgs::Float64>("/angle", 1000);
  
  // Initialize the subscriber for PWM data reception
  ros::Subscriber pwm_sub = n.subscribe("control_effort", 1000, PWMCallback);

  ros::Rate loop_rate(6000); 

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

  std_msgs::Float64 state_msg;
  state_msg.data = 0.0;

  while (ros::ok())
  {
 
    // If we got a new message, publish to topic and print values every 100 messages
    if(gotMsg){
    msg_count++;
      state_msg.data = (float) pos;

      position_interface_pub.publish(state_msg);
      
    if(msg_count >= 2000)
    {
    msg_count = 0;
    //ROS_INFO("%i  angle", pos);
    }
      gotMsg = false;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
