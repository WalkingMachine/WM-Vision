/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 06/04/2013
 *
 * Programmer: Keaven Martin
 *
 * Description:
 *
 */

#include "../../../include/vision_kernel/vision_nodes/parallel_lines_node.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>

Data ParallelLinesNode::Function(InputData input_data) {
  std::shared_ptr<Data> data = input_data->at("input");
  Data output_data;

  double angle = boost::lexical_cast<double>(parameters()["Angle"]) / 180 * CV_PI;
//  if(angle > CV_PI) angle -= CV_PI;

  double tolerance = boost::lexical_cast<double>(parameters()["AngleTolerance"]) / 180 * CV_PI;

  bool positif_tolerance_pass_pi = false;
  double positif_tolerance = (angle + tolerance);
  if(positif_tolerance >= CV_PI) {
	  positif_tolerance_pass_pi = true;
	  positif_tolerance -= CV_PI;
  }

  bool negatif_tolerance_pass_pi = false;
  double negatif_tolerance = (angle - tolerance);
  if(negatif_tolerance < 0) {
	  negatif_tolerance_pass_pi = true;
	  negatif_tolerance += CV_PI;
  }

  if (parameters()["IsOnCUDA"] == "true") {

  } else {
//	  std::shared_ptr<cv::vector<cv::Vec4i>> output_vector(new cv::vector<cv::Vec4i>);
	  cv::vector<cv::Vec4i> *lines = &*data->data<cv::vector<cv::Vec4i>>();
	    std::shared_ptr<cv::Mat> output_image(
	        new cv::Mat(480,
	                    640,
	                    CV_8U));
	  cv::vector<cv::Vec4i> output_vector;

	  float line_angle;

	  for(auto &line: *lines) {  //each lines
		  line_angle = cv::fastAtan2(line[1] - line[3], line[0] - line[2]) / 180 * CV_PI;  // Get line angle

		  if(line_angle >= CV_PI) line_angle -= CV_PI;

		  if(negatif_tolerance <= line_angle && line_angle <= positif_tolerance) {  // Verify angle with tolerance
			  output_vector.push_back(line);
		  }
	  }
//	  output_data.set_data(output_vector);

	output_image->setTo(0);  // Black image
	for(size_t i = 0; i < output_vector.size(); i++)
	{
	  cv::line(*output_image, cv::Point(output_vector[i][0], output_vector[i][1]),
				 cv::Point(output_vector[i][2], output_vector[i][3]), cv::Scalar(255), 1, 8);
    }
    output_data.set_data(output_image);
  }

  return output_data;
}
