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
 * Description: Outputs Object details via topic
 *
 */

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_OUTPUT_OBJECT_DETAIL_NODE_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_OUTPUT_OBJECT_DETAIL_NODE_H_

#include <ros/publisher.h>

#include "../data.h"
#include "../vision_node.h"
#include "../vision_node_factory.h"

class OutputObjectDetailNode : public VisionNode {
 public:
  OutputObjectDetailNode() {};
  virtual ~OutputObjectDetailNode();

  Data Function(InputData input_data);
  void Init();

 private:
  ros::Publisher publisher_;
  REGISTER_DEC_TYPE(OutputObjectDetailNode);
};

REGISTER_DEF_TYPE(OutputObjectDetailNode);

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_OUTPUT_OBJECT_DETAIL_NODE_H_
