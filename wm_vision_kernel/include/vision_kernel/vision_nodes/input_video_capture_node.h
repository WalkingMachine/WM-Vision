/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: vision_kernel
 *
 * Creation date: 12/08/2013
 *
 * Programmer: Keaven Martin
 *
 * Description: Grab video image from a video file
 *
 */

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_INPUT_VIDEO_CAPTURE_NODE_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_INPUT_VIDEO_CAPTURE_NODE_H_

#include "../vision_node.h"
#include "../data.h"
#include "../vision_node_factory.h"
#include <opencv2/opencv.hpp>

class InputVideoCaptureNode : public VisionNode {
 public:
  virtual ~InputVideoCaptureNode();
  Data Function(InputData input_data);
  void Init();

 private:
  cv::VideoCapture *video;
  REGISTER_DEC_TYPE(InputVideoCaptureNode);
};

REGISTER_DEF_TYPE(InputVideoCaptureNode);

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_INPUT_VIDEO_CAPTURE_NODE_H_
