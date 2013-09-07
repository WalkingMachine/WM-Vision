/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 15/05/2013
 *
 * Programmer: Keaven Martin
 *
 * Description:
 *
 */

#include "../../../include/vision_kernel/vision_nodes/different_color_betwen_door_and_wall_node.h"

#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>

enum {X1, Y1, X2, Y2};

Data DifferentColorBetwenDoorAndWallNode::Function(InputData input_data) {
  //Inputs
  std::shared_ptr<cv::Mat> image_rgb =
      input_data->at("imageRGB")->DataWithValidation<cv::Mat>();

  std::shared_ptr<std::deque<std::pair<cv::Vec4i, cv::Vec4i>>> line_pairs =
        input_data->at("verticalLinePairs")->DataWithValidation<std::deque<std::pair<cv::Vec4i, cv::Vec4i>>>();

  //Ouputs
  Data output_data;
  std::shared_ptr<std::deque<std::pair<cv::Vec4i, cv::Vec4i>>> output_line_pairs(new std::deque<std::pair<cv::Vec4i, cv::Vec4i>>);
  std::shared_ptr<cv::Mat> output_image(new cv::Mat(*image_rgb));
  char color = 0;

  //Parameters
  ValidationType validationType = StringToValidationType(parameters()["ValidationType"]);

  if (parameters()["IsOnCUDA"] == "true") {

  } else {
    cv::Mat average_color_right(cv::Size(1,1), CV_8UC3),
            average_color_center(cv::Size(1,1), CV_8UC3),
            average_color_left(cv::Size(1,1), CV_8UC3),
            average_lab_color_right,
            average_lab_color_center,
            average_lab_color_left;
    double right_vs_center_delta_e, left_vs_center_delta_e;
    int *lowest_x_first_line, *bigest_x_first_line,
        *lowest_x_second_line, *bigest_x_second_line;
    int *top_y, *bottom_y;
    int pixel_threshold = boost::lexical_cast<int>(parameters()["NbrPixelLeftOrRightOfDoor"]);
    double delta_e_max = 0.1;//boost::lexical_cast<double>(parameters()["DeltaEMax"]);
    int door_height;
    bool ok;

    for(auto &line_pair : *line_pairs) {
      ok = true;

      if (line_pair.first[X1] < line_pair.first[X2]) {
        lowest_x_first_line = &line_pair.first[X1];
        bigest_x_first_line = &line_pair.first[X2];
      } else {
        lowest_x_first_line = &line_pair.first[X2];
        bigest_x_first_line = &line_pair.first[X1];
      }

      if (line_pair.second[X1] < line_pair.second[X2]) {
        lowest_x_second_line = &line_pair.second[X1];
        bigest_x_second_line = &line_pair.second[X2];
      } else {
        lowest_x_second_line = &line_pair.second[X2];
        bigest_x_second_line = &line_pair.second[X1];
      }

      if (line_pair.first[Y1] < line_pair.second[Y1])
        top_y = &line_pair.second[Y1];
      else
        top_y = &line_pair.first[Y1];

      if (line_pair.first[Y2] < line_pair.second[Y2])
        bottom_y = &line_pair.first[Y2];
      else
        bottom_y = &line_pair.second[Y2];

      door_height = *bottom_y - *top_y;

      //Center of door
      cv::Mat3b center_image_rgb_masked = (*image_rgb)(cv::Rect(*bigest_x_first_line, *top_y, *lowest_x_second_line - *bigest_x_first_line,  door_height));

      average_color_center = cv::mean(center_image_rgb_masked);  // Find average color in center
      cv::cvtColor(average_color_center, average_lab_color_center, CV_RGB2Lab);  //Convert RGB to CIELAB

      // LEFT_VS_CENTER or ALL
      if(validationType != RIGHT_VS_CENTER) {
        // Left of door
        int x = *lowest_x_first_line - pixel_threshold;
        if(x < 0) x = 0;

        cv::Mat3b left_image_rgb_masked = (*image_rgb)(cv::Rect(x, *top_y, pixel_threshold, door_height));
        average_color_left = cv::mean(left_image_rgb_masked);  // Find average color at left
        cv::cvtColor(average_color_left, average_lab_color_left, CV_RGB2Lab);  //Convert RGB to CIELAB

        left_vs_center_delta_e = DeltaE(average_lab_color_left.data[0],
                                        average_lab_color_left.data[1],
                                        average_lab_color_left.data[2],
                                        average_lab_color_center.data[0],
                                        average_lab_color_center.data[1],
                                        average_lab_color_center.data[2]);

        if (left_vs_center_delta_e < delta_e_max) ok = false;
      }

      // RIGHT_VS_CENTER or ALL
      if (validationType != LEFT_VS_CENTER) {
        // Right of door
        int temp_pixel_threshold;
        if (pixel_threshold >= image_rgb->size().width) {
          temp_pixel_threshold = image_rgb->size().width;
        } else {
          temp_pixel_threshold = pixel_threshold;
        }
        cv::Mat3b right_image_rgb_masked = (*image_rgb)(cv::Rect(*bigest_x_second_line, *top_y, temp_pixel_threshold, door_height));
        average_color_right = cv::mean(right_image_rgb_masked); // Find average color at right
        cv::cvtColor(average_color_right, average_lab_color_right, CV_RGB2Lab); //Convert RGB to CIELAB

        right_vs_center_delta_e = DeltaE(average_lab_color_right.data[0],
                                         average_lab_color_right.data[1],
                                         average_lab_color_right.data[2],
                                         average_lab_color_center.data[0],
                                         average_lab_color_center.data[1],
                                         average_lab_color_center.data[2]);

        if (right_vs_center_delta_e < delta_e_max) ok = false;
      }

      if (ok) {
        output_line_pairs->push_back(line_pair);
        cv::rectangle(*output_image, cv::Rect(*bigest_x_first_line, *top_y, *lowest_x_second_line - *bigest_x_first_line,  door_height), cv::Scalar(20 + color, 150 + color, 200 + color));
        color += 42;
      }
    }

    output_data.set_data(output_image);
//	  output_data.set_data(output_line_pairs);
  }

  return output_data;
}

double DifferentColorBetwenDoorAndWallNode::DeltaE(
    double L1, double a1, double b1, double L2, double a2, double b2) {
  double delta_L = L2 - L1;
  double delta_a = a2 - a1;
  double delta_b = b2 - b1;

  return cv::sqrt(delta_L * delta_L + delta_a * delta_a + delta_b * delta_b);
}

DifferentColorBetwenDoorAndWallNode::ValidationType DifferentColorBetwenDoorAndWallNode::StringToValidationType(std::string validationType) {
  if("LEFT_VS_CENTER")
    return LEFT_VS_CENTER;
  else if ("RIGHT_VS_CENTER")
    return RIGHT_VS_CENTER;
  else
    return ALL;
}
