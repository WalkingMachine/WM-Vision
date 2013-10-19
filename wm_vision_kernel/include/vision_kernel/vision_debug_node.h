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

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_DEBUG_NODE_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_DEBUG_NODE_H_

#include <string>
#include <map>
#include <memory>

#include "vision_node.h"

class VisionDebugNode : public VisionNode {
 public:
  // Constructor and Destructor
  VisionDebugNode();
  virtual ~VisionDebugNode();

  void set_vision_node(std::shared_ptr<VisionNode> vision_node);

  void CallbackFunction(std::shared_ptr<VisionNode> vision_node,
                        std::shared_ptr<Data> output_data);
  virtual Data Function(InputData input_data) = 0;
  virtual void Init() {};

  void Thread();

 private:
  std::shared_ptr<VisionNode> vision_node_;
};

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_VISION_DEBUG_NODE_H_
