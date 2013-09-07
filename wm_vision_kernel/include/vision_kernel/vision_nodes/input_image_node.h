/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: vision_kernel
 *
 * Creation date: 01/15/2013
 *
 * Programmer: Keaven Martin
 *
 * Description: Connect with openni_camera and
 *              output camera data or information
 *
 */

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_INPUT_IMAGE_NODE_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_INPUT_IMAGE_NODE_H_

#include "../vision_node.h"
#include "../data.h"
#include "../vision_node_factory.h"
#include "../input_manager.h"

class InputImageNode : public VisionNode {
 public:
  virtual ~InputImageNode();
  Data Function(InputData input_data);
  void Init();

 private:
  REGISTER_DEC_TYPE(InputImageNode);
};

REGISTER_DEF_TYPE(InputImageNode);

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_INPUT_IMAGE_NODE_H_
