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
 * Description: Outputs gesture position via topic
 *
 */

#ifndef OUTPUT_GESTURE_NODE_H_
#define OUTPUT_GESTURE_NODE_H_

#include <ros/publisher.h>

#include "../data.h"
#include "../vision_node.h"
#include "../vision_node_factory.h"

class OutputGestureNode : public VisionNode {
 public:
  OutputGestureNode();
  virtual ~OutputGestureNode();

  Data Function(InputData input_data);
  void Init();

 private:
  ros::Publisher publisher_;
  REGISTER_DEC_TYPE(OutputGestureNode);
};

  REGISTER_DEF_TYPE(OutputGestureNode);

#endif /* OUTPUT_GESTURE_NODE_H_ */
