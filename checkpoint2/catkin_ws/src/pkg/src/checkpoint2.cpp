#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>

using namespace std;


int main(int argc ,char **argv){
 ros::init(argc,argv,"checkpoint_2_pub");
 ros::NodeHandle nh;
 ros::Publisher number_publisher=nh.advertise<std_msgs::Int32>("array",1);
 ros::Rate loop_rate(10);

 ros::Duration(2).sleep();
 std_msgs::Int32 msg;

 int n=0;
 while(ros::ok() ){
  int i;  
  if(n==1){cout<<"input a num:(r)";}
  if(n==2){cout<<"input a num:(l)";}
  if(n==0){cout<<"input a num:(MODE)";}
  n++;
  cin>>i;
  msg.data=i;
  if(n==3){n=0;}
  if(i== 256){break;}

 

  number_publisher.publish(msg);
  cout<<"PUB";
  ros::spinOnce();
  loop_rate.sleep();
 }
}

