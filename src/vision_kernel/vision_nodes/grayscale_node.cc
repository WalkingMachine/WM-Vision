/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: vision_kernel
 *
 * Creation date: 12/07/2012
 *
 * Programmer: Keaven Martin
 *
 * Description:
 *
 */

#include "../../../include/vision_kernel/vision_nodes/grayscale_node.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>

/**
 * Base node function (Transfert an RGB(CV_8U) image in grayscale(CV_1U))
 * @param input_data
 * @return input_data
 */
Data GrayscaleNode::Function(InputData input_data) {
  std::shared_ptr<Data> data = input_data->at("input");
  Data output_data;

  if (parameters()["IsOnCUDA"] == "true") {
    std::shared_ptr<cv::gpu::GpuMat> gpu_output_image(
        new cv::gpu::GpuMat(data->data<cv::gpu::GpuMat>()->cols,
                            data->data<cv::gpu::GpuMat>()->rows,
                            CV_MAKETYPE(data->data<cv::gpu::GpuMat>()->depth(), 1)));
    cv::gpu::cvtColor(*data->data<cv::gpu::GpuMat>(),
                      *gpu_output_image,
                      GrayscaleTypeStringToInt(parameters()["GrayscaleType"]));
    output_data.set_data(gpu_output_image);
  } else {
    std::shared_ptr<cv::Mat> output_image(
        new cv::Mat(data->data<cv::Mat>()->cols,
                    data->data<cv::Mat>()->rows,
                    CV_MAKETYPE(data->data<cv::Mat>()->depth(), 1)));
    cv::cvtColor(*data->data<cv::Mat>(),
                 *output_image,
                 GrayscaleTypeStringToInt(parameters()["GrayscaleType"]));
    output_data.set_data(output_image);
  }

  return output_data;
}

int GrayscaleNode::GrayscaleTypeStringToInt(const std::string &grayscale_type) {
  int grayscale_type_int = CV_BGR2GRAY;

  if(grayscale_type == "CV_BGR2GRAY")
    grayscale_type_int = CV_BGR2GRAY;
  else if(grayscale_type == "CV_RGB2GRAY")
    grayscale_type_int = CV_RGB2GRAY;
  else if(grayscale_type == "CV_BGRA2GRAY")
    grayscale_type_int = CV_BGRA2GRAY;
  else if(grayscale_type == "CV_RGBA2GRAY")
    grayscale_type_int = CV_RGBA2GRAY;

  return grayscale_type_int;
}
