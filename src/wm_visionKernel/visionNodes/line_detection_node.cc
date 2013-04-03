/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 03/18/2013
 *
 * Programmer: Keaven Martin
 *
 * Description: Image strait line detection
 *
 */

#include "../include/wm_visionKernel/visionNodes/line_detection_node.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>

Data LineDetectionNode::Function(std::map<std::string, Data>  *input_data) {
  Data *data = &input_data->at("input");
  Data output_data;

  if (parameters()["IsOnCUDA"] == "true") {

  } else {
	//Create output ...
	cv::vector<cv::Vec4i> lines;
    std::shared_ptr<cv::Mat> output_image(
        new cv::Mat(data->data<cv::Mat>()->cols,
                    data->data<cv::Mat>()->rows,
                    CV_MAKETYPE(data->data<cv::Mat>()->depth(), 1)));

    cv::HoughLinesP(*data->data<cv::Mat>(), lines, 1, CV_PI/180, 80, 30, 10);

    output_image->setTo(cv::Scalar(0));
    for(size_t i = 0; i < lines.size(); i++)
	{
		cv::line(*output_image, cv::Point(lines[i][0], lines[i][1]),
				 cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255), 3, 8);
	}

    output_data.set_data(output_image);
  }

  return output_data;
}
