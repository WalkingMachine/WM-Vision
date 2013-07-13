/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: vision_kernel
 *
 * Creation date: 26/02/2013
 *
 * Programmer: Keaven Martin
 *
 * Description: Output debug data (EX: via ROS Topic)
 *
 */

#include "../../include/vision_kernel/vision_debug_node.h"

#include <ros/ros.h>

VisionDebugNode::VisionDebugNode() {
  vision_node_ = nullptr;
}

VisionDebugNode::~VisionDebugNode() {

}

void VisionDebugNode::set_vision_node(std::shared_ptr<VisionNode> vision_node) {
  vision_node_ = vision_node;
}

void VisionDebugNode::CallbackFunction(std::shared_ptr<VisionNode> vision_node,
                                       std::shared_ptr<Data> output_data) {
  bool debug;

  ros::NodeHandle node_handle;

  node_handle.getParamCached("debug_mode", debug);

  if (debug) {
    InputData data(new std::map<std::string, std::shared_ptr<Data>>);
    data->insert(std::pair<std::string, std::shared_ptr<Data>>("input", output_data));
    Function(data);
  }

  call_flow_callback_function(vision_node, output_data);
}

/**
 * Debug Node thread
 * @param input_data
 */
void VisionDebugNode::Thread() {
  do {
    boost::mutex::scoped_lock lock(thread_mutex_);
    vision_node_->StartOneIteration(input_data_);

    thread_start_condition_.wait(lock);
  } while(!must_stop_);
  thread_stoped_condition_.notify_one();
}
