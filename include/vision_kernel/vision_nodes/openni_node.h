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

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_OPENNI_NODE_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_OPENNI_NODE_H_

#include "../vision_node.h"
#include "../data.h"
#include "../vision_node_factory.h"
#include "../input_manager.h"

class OpenniNode : public VisionNode {
 public:
  virtual ~OpenniNode();
  Data Function(InputData input_data);
  void Init();

 private:
  //InputManager<sensor_msgs::Image> input_manager_image_;
  REGISTER_DEC_TYPE(OpenniNode);
};

REGISTER_DEF_TYPE(OpenniNode);

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_OPENNI_NODE_H_
