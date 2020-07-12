/*
 * Copyright [2011] <Bonn-Rhein-Sieg University>
 *
 * teleop_joypad_node.cpp
 *
 *  Created on: Dec 15, 2011
 *      Author: Frederik Hegger
 */

#include "mir_teleop/teleop_joypad.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_joypad");
  ros::NodeHandle nh("~");

  TeleOpJoypad *teleop = new TeleOpJoypad(nh);

  ros::Rate loop_rate(50);

  ros::spin();

  delete teleop;
}
