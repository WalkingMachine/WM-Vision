/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision_kernel
 *
 * Creation date: 04/13/2013
 *
 * Programmer: Julien Côté, Keaven Martin
 *
 * Description: Main function of vision kernel
 *
 */

#include <ros/ros.h>

#include "../include/vision_kernel/vision_interface.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "wm_vision_kernel");
  ros::NodeHandle node_handle;
  VisionInterface interface;

  ros::ServiceServer service = node_handle.advertiseService(
      ros::this_node::getName(),
      &VisionInterface::CallbackFlow,
      &interface);

  ROS_INFO("wm_vision_kernel initialized");
  ros::spin();

  return 0;
}
