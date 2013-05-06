/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: vision_kernel
 *
 * Creation date: 03/18/2013
 *
 * Programmer: Keaven Martin
 *
 * Description: Image edge detection
 *
 */

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_EDGE_DETECTION_NODE_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_EDGE_DETECTION_NODE_H_

#include "../vision_node.h"
#include "../data.h"
#include "../vision_node_factory.h"

class EdgeDetectionNode: public VisionNode {
 public:
  Data Function(InputData input_data);

 private:
  REGISTER_DEC_TYPE(EdgeDetectionNode);
};

REGISTER_DEF_TYPE(EdgeDetectionNode);

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_EDGE_DETECTION_NODE_H_
