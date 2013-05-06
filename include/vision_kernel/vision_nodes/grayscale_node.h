/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: vision_kernel
 *
 * Creation date: 12/07/2012
 *
 * Programmer: Keaven Martin
 *
 * Description: Grayscale vision filter (Temporary node)
 *
 */

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_GRAYSCALE_NODE_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_GRAYSCALE_NODE_H_

#include <string>

#include "../vision_node.h"
#include "../data.h"
#include "../vision_node_factory.h"

class GrayscaleNode: public VisionNode {
 public:
  Data Function(InputData input_data);

 private:
  REGISTER_DEC_TYPE(GrayscaleNode);
  int GrayscaleTypeStringToInt(const std::string &threshold_type);
};

REGISTER_DEF_TYPE(GrayscaleNode);

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_GRAYSCALE_NODE_H_
