/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: vision_kernel
 *
 * Creation date: 02/25/2013
 *
 * Programmer: Julien Côté
 *
 * Description: OpenCV threshold vision filter
 *
 */

#include "../../../include/vision_kernel/vision_nodes/threshold_node.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <boost/lexical_cast.hpp>

/**
 * Base node function (Transfert an RGB(CV_8U) image in grayscale(CV_1U))
 * @param input_data
 * @return input_data
 */
Data ThresholdNode::Function(InputData input_data) {
  std::shared_ptr<Data> data = input_data->at("input");
  Data output_data;

  if (parameters()["IsOnCUDA"] == "true") {
    std::shared_ptr<cv::gpu::GpuMat> gpu_output_image(
        new cv::gpu::GpuMat(data->data<cv::gpu::GpuMat>()->cols,
                            data->data<cv::gpu::GpuMat>()->rows,
                            CV_MAKETYPE(data->data<cv::gpu::GpuMat>()->depth(), 1)));

    cv::gpu::threshold(*data->data<cv::gpu::GpuMat>(),
                       *gpu_output_image,
                       boost::lexical_cast<double>(parameters()["Threshold"]),
                       boost::lexical_cast<double>(parameters()["MaximumValue"]),
                       ThresholdTypeStringToInt(parameters()["ThresholdType"]));

    output_data.set_data(gpu_output_image);
  } else {
    std::shared_ptr<cv::Mat> output_image(
        new cv::Mat(data->data<cv::Mat>()->cols,
                    data->data<cv::Mat>()->rows,
                    CV_MAKETYPE(data->data<cv::Mat>()->depth(), 1)));

    cv::threshold(*data->data<cv::Mat>(),
                  *output_image,
                  boost::lexical_cast<double>(parameters()["Threshold"]),
                  boost::lexical_cast<double>(parameters()["MaximumValue"]),
                  ThresholdTypeStringToInt(parameters()["ThresholdType"]));

    output_data.set_data(output_image);
  }

  return output_data;
}

int ThresholdNode::ThresholdTypeStringToInt(const std::string &threshold_type) {
  int threshold_type_int = cv::THRESH_BINARY;

  if(threshold_type == "THRESH_BINARY")
    threshold_type_int = cv::THRESH_BINARY;
  else if(threshold_type == "THRESH_BINARY_INV")
    threshold_type_int = cv::THRESH_BINARY_INV;
  else if(threshold_type == "THRESH_TRUNC")
    threshold_type_int = cv::THRESH_TRUNC;
  else if(threshold_type == "THRESH_TOZERO")
    threshold_type_int = cv::THRESH_TOZERO;
  else if(threshold_type == "THRESH_TOZERO_INV")
    threshold_type_int = cv::THRESH_TOZERO_INV;

  return threshold_type_int;
}
