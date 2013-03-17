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
 * Description: Outputs Object boundingbox via topic
 *
 */

#include "../include/wm_visionKernel/visionNodes/output_object_boundingbox_node.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

Data OutputObjectBoundingboxNode::Function(std::map<std::string, Data>  *input_data) {
  Data *data = &input_data->at("input");
  Data output_data;
  std_msgs::Header header;


  cv_bridge::CvImagePtr output_message(
      new cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, *data->data<cv::Mat>()));
  publisher_.publish(output_message);

  return output_data;
}

void OutputObjectBoundingboxNode::Init(){
  ros::NodeHandle node_handle;
  std::string topic_name = tree_name();

  publisher_ =  node_handle.advertise<sensor_msgs::Image>(topic_name, 30);
}

OutputObjectBoundingboxNode::~OutputObjectBoundingboxNode() {
  publisher_.shutdown();
}
