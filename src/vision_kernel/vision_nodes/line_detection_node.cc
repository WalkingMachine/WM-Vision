/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: vision_kernel
 *
 * Creation date: 03/18/2013
 *
 * Programmer: Keaven Martin
 *
 * Description: Image strait line detection
 *
 */

#include "../../../include/vision_kernel/vision_nodes/line_detection_node.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <boost/lexical_cast.hpp>

Data LineDetectionNode::Function(InputData input_data) {
  std::shared_ptr<Data> data = input_data->at("input");
  Data output_data;

  if (parameters()["IsOnCUDA"] == "true") {

  } else {
	std::shared_ptr<cv::vector<cv::Vec4i>> output_vector(new cv::vector<cv::Vec4i>);
	cv::vector<cv::Vec4i> lines;
    std::shared_ptr<cv::Mat> output_image(
        new cv::Mat(data->data<cv::Mat>()->rows,
                    data->data<cv::Mat>()->cols,
                    CV_8U));

    cv::HoughLinesP(*data->data<cv::Mat>(),
    		        *output_vector,
    		        boost::lexical_cast<double>(parameters()["Rho"]),
    		        CV_PI/boost::lexical_cast<double>(parameters()["Theta"]),
    		        boost::lexical_cast<int>(parameters()["Threshold"]),
    		        boost::lexical_cast<double>(parameters()["MinLineLength"]),
    		        boost::lexical_cast<double>(parameters()["MaxLineGap"]));

//    output_image->setTo(0);  // Black image
//    for(size_t i = 0; i < lines.size(); i++) {
//		cv::line(*output_image, cv::Point(lines[i][0], lines[i][1]),
//				 cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255), 1, 8);
//	  }

    output_data.set_data(output_vector);
  }

  return output_data;
}
