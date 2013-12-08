/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: vision_kernel
 *
 * Creation date: 03/18/2013
 *
 * Programmer: Julien Côté
 *
 * Description: Image strait line detection
 *
 */

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_OBJECT_DETECTION_NODE_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_OBJECT_DETECTION_NODE_H_

#include <string>
#include <map>

#include "../vision_node.h"
#include "../data.h"
#include "../vision_node_factory.h"

class ObjectDetectionNode: public VisionNode {
 public:
  Data Function(InputData input_data);

 private:
  REGISTER_DEC_TYPE(ObjectDetectionNode);
};

REGISTER_DEF_TYPE(ObjectDetectionNode);

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_OBJECT_DETECTION_NODE_H_
