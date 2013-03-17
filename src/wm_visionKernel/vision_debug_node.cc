/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 26/02/2013
 *
 * Programmer: Keaven Martin
 *
 * Description: Output debug data (EX: via ROS Topic)
 *
 */

#include "../include/wm_visionKernel/vision_debug_node.h"
#include <ros/ros.h>

VisionDebugNode::VisionDebugNode() {
  vision_node_ = nullptr;
}

VisionDebugNode::~VisionDebugNode() {

}

void VisionDebugNode::set_vision_node(VisionNode* vision_node) {
  vision_node_ = vision_node;
}

void VisionDebugNode::CallbackFunction(VisionNode* vision_node, Data output_data) {
  bool debug;

  ros::NodeHandle node_handle;

  node_handle.getParamCached("debug_mode", debug);

  if(debug) {
    std::map<std::string, Data> data;
    data.insert(std::pair<std::string, Data>("input",output_data));
    Function(&data);
  }

  call_tree_callback_function(vision_node, output_data);
}

/**
 * Debug Node thread
 * @param input_data
 */
void VisionDebugNode::Thread(std::map<std::string, Data> *input_data) {
  vision_node_->StartOneIteration(input_data);
}
