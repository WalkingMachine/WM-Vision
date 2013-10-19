/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision_kernel
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

/**
 * Constructor
 */
VisionDebugNode::VisionDebugNode() {
  vision_node_ = nullptr;
}

/**
 * Destructor
 */
VisionDebugNode::~VisionDebugNode() {

}

/**
 * Set vision node attached to this vision debuf node
 * @param vision_node
 */
void VisionDebugNode::set_vision_node(std::shared_ptr<VisionNode> vision_node) {
  vision_node_ = vision_node;
}

/**
 * Call back function used by the vision node when the primary function is finished
 * (only use when a debug node is linked to a vision node)
 * @param vision_node
 * @param output_data
 */
void VisionDebugNode::CallbackFunction(std::shared_ptr<VisionNode> vision_node,
                                       std::shared_ptr<Data> output_data) {
  bool debug;

  ros::NodeHandle node_handle;

  if (!node_handle.getParamCached("debug_mode", debug))
	  debug = false;  // Temporary at true for debugging without roslaunch

  if (debug) {
    InputData data(new std::map<std::string, std::shared_ptr<Data>>);
    data->insert(std::pair<std::string, std::shared_ptr<Data>>("input", output_data));
    Function(data);
  }

  call_flow_callback_function(vision_node, output_data);
}

/**
 * Debug Node thread
 */
void VisionDebugNode::Thread() {
  do {
    boost::mutex::scoped_lock lock(thread_mutex_);
    vision_node_->StartOneIteration(input_data_);

    thread_start_condition_.wait(lock);
  } while(!must_stop_);
  thread_stoped_condition_.notify_one();
}
