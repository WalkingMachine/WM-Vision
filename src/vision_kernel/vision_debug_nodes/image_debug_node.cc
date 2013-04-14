/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 01/15/2013
 *
 * Programmer: Keaven Martin
 *
 * Description: Connect with openni_camera and
 *              output camera data or information
 *
 */

#include "../../../include/vision_kernel/vision_debug_nodes/image_debug_node.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

Data ImageDebugNode::Function(std::map<std::string, Data>  *input_data) {
  Data *data = &input_data->at("input");

  std_msgs::Header header;
      int encode_number = data->data<cv::Mat>()->type();
      std::string encoding;

      switch(encode_number){
        case CV_8U:
          encoding = sensor_msgs::image_encodings::MONO8;
          break;
        case CV_8UC3:
          encoding = sensor_msgs::image_encodings::BGR8;
          break;
        default:
          ROS_ERROR("Invalid image encoding in %s debug node", id().c_str());
          break;
      }

      cv_bridge::CvImagePtr output_message(
          new cv_bridge::CvImage(header, encoding, *data->data<cv::Mat>()));

  publisher_.publish(output_message);

  Data empty_data;
  return empty_data;
}

void ImageDebugNode::Init(){
  ros::NodeHandle node_handle;
  std::string topic_name = tree_name() + "/" + id();

  publisher_= node_handle.advertise<sensor_msgs::Image>(topic_name, 30);
}

ImageDebugNode::~ImageDebugNode() {
  publisher_.shutdown();
}
