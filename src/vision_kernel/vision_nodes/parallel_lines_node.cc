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
#include <opencv2/gpu/gpu.hpp>
#include <boost/lexical_cast.hpp>

enum {X1, Y1, X2, Y2};

Data ParallelLinesNode::Function(InputData input_data) {
  std::shared_ptr<cv::vector<cv::Vec4i>> lines =
      input_data->at("lines")->DataWithValidation<cv::vector<cv::Vec4i>>();


  // Get offset if you received it in input data
  double offset_angle = 0;
  std::map<std::string, std::shared_ptr<Data>>::iterator offset_angle_data;

  offset_angle_data = input_data->find("offsetAngle");
  if ((offset_angle_data = input_data->find("offsetAngle")) != input_data->end()) {
    offset_angle = *offset_angle_data->second->DataWithValidation<double>();
  }

  Data output_data;

  double angle =
      (boost::lexical_cast<double>(parameters()["Angle"]) + offset_angle) / 180 * CV_PI;
  if(angle > CV_PI) angle -= CV_PI;

  double tolerance =
      boost::lexical_cast<double>(parameters()["AngleTolerance"]) / 180 * CV_PI;

  // Positif tolerance
  bool positif_tolerance_pass_pi = false;
  double positif_tolerance = (angle + tolerance);
  if(positif_tolerance >= CV_PI) {
	  positif_tolerance_pass_pi = true;
	  positif_tolerance -= CV_PI;
  }

  //Negatif tolerance
  bool negatif_tolerance_pass_pi = false;
  double negatif_tolerance = (angle - tolerance);
  if(negatif_tolerance < 0) {
	  negatif_tolerance_pass_pi = true;
	  negatif_tolerance += CV_PI;
  }

  if (parameters()["IsOnCUDA"] == "true") {

  } else {
	  std::shared_ptr<cv::vector<cv::Vec4i>> output_vector(new cv::vector<cv::Vec4i>);

	  float line_angle;
	  bool positif_tolerance_ok, negatif_tolerance_ok;

	  for (auto &line: *lines) {  // Each lines
		  line_angle =
		      cv::fastAtan2(line[X1] - line[X2], line[Y1] - line[Y2]) / 180 * CV_PI;  // Get line angle
		  if(line_angle >= CV_PI) line_angle -= CV_PI;

      // Positif tolerance validation
      if (positif_tolerance_pass_pi) {
        positif_tolerance_ok =
            (line_angle <= positif_tolerance) || (line_angle >= angle);
      } else {
        positif_tolerance_ok =
            (line_angle <= positif_tolerance) && (line_angle >= angle);
      }

      // Negatif tolerance validation
      if (negatif_tolerance_pass_pi) {
        negatif_tolerance_ok =
            (line_angle >= negatif_tolerance) || (line_angle <= angle);
      } else {
        negatif_tolerance_ok =
            (line_angle >= negatif_tolerance) && (line_angle <= angle);
      }

		  if (negatif_tolerance_ok || positif_tolerance_ok)
		    output_vector->push_back(line);
	  }

	  output_data.set_data(output_vector);
  }

  return output_data;
}
