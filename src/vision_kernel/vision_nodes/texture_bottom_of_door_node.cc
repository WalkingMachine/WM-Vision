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

#include "../../../include/vision_kernel/vision_nodes/texture_bottom_of_door_node.h"

#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <math.h>

enum {X1, Y1, X2, Y2};

Data TextureBottomOfDoorNode::Function(InputData input_data) {
  //Inputs
  std::shared_ptr<cv::Mat> image_rgb =
      input_data->at("edgeImage")->DataWithValidation<cv::Mat>();

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
    cv::Scalar average_color_right, average_color_center, average_color_left;
    double delta_right_vs_center, delta_left_vs_center;
    int *lowest_x_first_line, *bigest_x_first_line,
        *lowest_x_second_line, *bigest_x_second_line;
    int top_y;
    int *bottom_y;
    int number_pixel_bottom_of_door = boost::lexical_cast<int>(parameters()["NbrPixelBottomOfDoor"]);
    int pixel_threshold = boost::lexical_cast<int>(parameters()["NbrPixelLeftOrRightOfDoor"]);
    double delta_min = boost::lexical_cast<double>(parameters()["DeltaMin"]);
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

      if (line_pair.first[Y2] < line_pair.second[Y2])
        bottom_y = &line_pair.first[Y2];
      else
        bottom_y = &line_pair.second[Y2];


      if((top_y = *bottom_y - number_pixel_bottom_of_door) < 0) top_y =  0;

      if (*lowest_x_second_line - *bigest_x_first_line > 0) {  //Remove near angular parallel lines
        //Center of door

        cv::Mat center_image_rgb_masked = (*image_rgb)(cv::Rect(*bigest_x_first_line, top_y, *lowest_x_second_line - *bigest_x_first_line,  number_pixel_bottom_of_door));

        average_color_center = cv::mean(center_image_rgb_masked);  // Find average color in center

        // LEFT_VS_CENTER or ALL
        if(validationType != RIGHT_VS_CENTER) {
          // Left of door
          int x = *lowest_x_first_line - pixel_threshold;
          if(x < 0) x = 0;

          cv::Mat left_image_rgb_masked = (*image_rgb)(cv::Rect(x, top_y, pixel_threshold, number_pixel_bottom_of_door));
          average_color_left = cv::mean(left_image_rgb_masked);  // Find average color at left

          delta_left_vs_center = average_color_center.val[0] - average_color_left.val[0];

          if (delta_left_vs_center > delta_min) ok = false;
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

          cv::Mat right_image_rgb_masked = (*image_rgb)(cv::Rect(*bigest_x_second_line, top_y, temp_pixel_threshold, number_pixel_bottom_of_door));
          average_color_right = cv::mean(right_image_rgb_masked); // Find average color at right

          delta_right_vs_center = average_color_center.val[0] - average_color_right.val[0];

          if (delta_right_vs_center > delta_min) ok = false;
        }

        if (ok) {
          output_line_pairs->push_back(line_pair);
          cv::rectangle(*output_image, cv::Rect(*bigest_x_first_line, top_y, *lowest_x_second_line - *bigest_x_first_line,  number_pixel_bottom_of_door), cv::Scalar(20 + color, 150 + color, 200 + color));
          color += 42;
        }
      }
    }

    output_data.set_data(output_image);
//	  output_data.set_data(output_line_pairs);
  }

  return output_data;
}

TextureBottomOfDoorNode::ValidationType TextureBottomOfDoorNode::StringToValidationType(std::string validationType) {
  if("LEFT_VS_CENTER")
    return LEFT_VS_CENTER;
  else if ("RIGHT_VS_CENTER")
    return RIGHT_VS_CENTER;
  else
    return ALL;
}
