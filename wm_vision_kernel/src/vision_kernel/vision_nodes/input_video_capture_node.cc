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

#include "../../../include/vision_kernel/vision_nodes/input_video_capture_node.h"

#include <sensor_msgs/Image.h>

void InputVideoCaptureNode::Init() {
  video = new cv::VideoCapture(parameters()["VideoPath"]);
}

Data InputVideoCaptureNode::Function(InputData input_data) {
  std::shared_ptr<cv::Mat> input_image(new cv::Mat);
  cv::Mat test;

  int next_frame = video->get(CV_CAP_PROP_POS_FRAMES);
  int frame_count = video->get(CV_CAP_PROP_FRAME_COUNT);

  if (next_frame >= frame_count)
    video->set(CV_CAP_PROP_POS_FRAMES, 0);

  *video >> *input_image;

  Data output_data;
  output_data.set_data(input_image);

  return output_data;
}

InputVideoCaptureNode::~InputVideoCaptureNode() {
  video->release();
}
