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
 * Description: Outputs gesture position via topic
 *
 */

#include "../../../include/vision_kernel/vision_nodes/output_gesture_node.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

Data OutputGestureNode::Function(InputData input_data) {
  std::shared_ptr<Data> data = input_data->at("input");
  Data output_data;

  std_msgs::Header header;
  cv_bridge::CvImagePtr output_message(
      new cv_bridge::CvImage(header,
                             sensor_msgs::image_encodings::BGR8,
                             *data->data<cv::Mat>()));

  publisher_.publish(output_message);

  return output_data;
}

void OutputGestureNode::Init() {
  ros::NodeHandle node_handle;
  std::string topic_name= tree_name();

  publisher_ =  node_handle.advertise<sensor_msgs::Image>(topic_name, 30);
}

OutputGestureNode::~OutputGestureNode() {
  publisher_.shutdown();
}
