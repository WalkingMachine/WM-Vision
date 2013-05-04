/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: vision_kernel
 *
 * Creation date: 02/26/2013
 *
 * Programmer: Julien Côté
 *
 * Description: Outputs gesture position via topic
 *
 */

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_OUTPUT_GESTURE_NODE_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_OUTPUT_GESTURE_NODE_H_

#include <ros/publisher.h>

#include "../data.h"
#include "../vision_node.h"
#include "../vision_node_factory.h"

class OutputGestureNode : public VisionNode {
 public:
  OutputGestureNode() {};
  virtual ~OutputGestureNode();

  Data Function(InputData input_data);
  void Init();

 private:
  ros::Publisher publisher_;
  REGISTER_DEC_TYPE(OutputGestureNode);
};

REGISTER_DEF_TYPE(OutputGestureNode);

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_OUTPUT_GESTURE_NODE_H_
