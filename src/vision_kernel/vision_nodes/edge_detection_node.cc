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
 * Description: Image edge detection
 *
 */

#include "../../../include/vision_kernel/vision_nodes/edge_detection_node.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <boost/lexical_cast.hpp>


Data EdgeDetectionNode::Function(std::map<std::string, Data>  *input_data) {
  Data *data = &input_data->at("input");
  Data output_data;

  if (parameters()["IsOnCUDA"] == "true") {

  } else {
	  //Create output image
	  std::shared_ptr<cv::Mat> output_image(
	          new cv::Mat(data->data<cv::Mat>()->cols,
	                      data->data<cv::Mat>()->rows,
	                      CV_8U));

	  //Blur input image
	  cv::blur(*data->data<cv::Mat>(),
			   *output_image,
			   cv::Size(boost::lexical_cast<int>(parameters()["BlurKWidth"]),
					    boost::lexical_cast<int>(parameters()["BlurKHeight"])));

	  //Find edge
	  cv::Canny(*output_image,
			  	*output_image,
			    boost::lexical_cast<int>(parameters()["Threshold1"]),
			    boost::lexical_cast<int>(parameters()["Threshold2"]),
			    3,
			    parameters()["L2gradient"] == "true");
	  output_data.set_data(output_image);
  }

  return output_data;
}
