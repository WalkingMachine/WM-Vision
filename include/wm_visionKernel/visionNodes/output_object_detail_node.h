/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 02/26/2013
 *
 * Programmer: Julien Côté
 *
 * Description: Outputs Object details via topic
 *
 */

#ifndef OUTPUT_OBJECT_DETAIL_NODE_H_
#define OUTPUT_OBJECT_DETAIL_NODE_H_

#include <ros/publisher.h>

#include "../include/wm_visionKernel/data.h"
#include "../include/wm_visionKernel/vision_node.h"
#include "../include/wm_visionKernel/vision_node_factory.h"

class OutputObjectDetailNode : public VisionNode {
 public:
  OutputObjectDetailNode();
  virtual ~OutputObjectDetailNode();

  Data Function(std::map<std::string, Data> *input_data);
  void Init();

 private:
  ros::Publisher publisher_;
  REGISTER_DEC_TYPE(OutputObjectDetailNode);
};

  REGISTER_DEF_TYPE(OutputObjectDetailNode);

#endif /* OUTPUT_OBJECT_DETAIL_NODE_H_ */
