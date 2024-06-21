#include <Arduino.h>
#include <control.h>
#include <ros.h>
#include <std_msgs/Int16.h>


ros::NodeHandle nh;

void move(const std_msgs::Int16& cmd_msg)
{
  drive(cmd_msg.data);
}



void spot_move(const std_msgs::Int16& cmd_msg)
{
  spot_drive(cmd_msg.data);
}
ros::Subscriber<std_msgs::Int16> sub01("/move", move);
ros::Subscriber<std_msgs::Int16> sub02("/spot_turn", spot_move);



void setup() {

  initial_setup();
  nh.initNode();
  nh.subscribe(sub01);
  nh.subscribe(sub02);


}

void loop() {
  nh.spinOnce();
  if(nh.connected()){
  }
  delay(100);
}