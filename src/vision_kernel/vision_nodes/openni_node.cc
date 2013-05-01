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

#include "../../../include/vision_kernel/vision_nodes/openni_node.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>

void OpenniNode::Init() {
  //input_manager_image_ = InputManager<sensor_msgs::Image>::GetInstance();

  // TODO(Julien cote) Make sure ImageConstPtr is safe or change toImage
  InputManager<sensor_msgs::Image>::GetInstance().AddSubscriber(parameters()["TopicRGB"]);
}

/**
 * Base node function (Connect with openni_camera and
 * output camera data or information)
 * @param input_data
 * @return cv::Mat image
 */
Data OpenniNode::Function(InputData input_data) {
  std::shared_ptr<cv::Mat> input_image(new cv::Mat);

  while(!InputManager<sensor_msgs::Image>::GetInstance().has_data(parameters()["TopicRGB"])){
    ros::Duration(0.1).sleep();
  }

  sensor_msgs::Image image_message =
     InputManager<sensor_msgs::Image>::GetInstance().GetInput(parameters()["TopicRGB"]);
  cv_bridge::CvImagePtr cv_image;

  try {
    cv_image = cv_bridge::toCvCopy(image_message, "bgr8");;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    // TODO should return error
  }

 *input_image = cv_image->image;

  Data output_data;
  output_data.set_data(input_image);

  return output_data;
}

OpenniNode::~OpenniNode(){
  InputManager<sensor_msgs::ImageConstPtr>::GetInstance().RemoveSubscriber(parameters()["TopicRGB"]);
}

