/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: vision_kernel
 *
 * Creation date: 05/06/2013
 *
 * Programmer: Félix Thérien
 *
 * Description: Find door gap at the bottom of a close door
 *
 */

#include "../../../include/vision_kernel/vision_nodes/door_gap_node.h"

#include <math.h>
#include <deque>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <boost/lexical_cast.hpp>


Data DoorGapNode::Function(InputData input_data) {
  std::shared_ptr<cv::vector<cv::Vec4i>> horizontals_lines =
      input_data->at("horizontalsLinesVector")->DataWithValidation<cv::vector<cv::Vec4i>>();
  std::shared_ptr<std::deque<std::pair<cv::Vec4i*, cv::Vec4i*>>> verticals_lines_pairs =
      input_data->at("verticalsLinesDeque")->DataWithValidation<std::deque<std::pair<cv::Vec4i*, cv::Vec4i*>>>();
  //catkin_make_eclipse
  Data output_data;

  //Gray screen matrice
  std::shared_ptr<cv::Mat> screenImg(
      new cv::Mat(cv::Size(640,480),CV_8UC3));
  cv::cvtColor(*screenImg, *screenImg, CV_BGR2GRAY);

  // Temporary lines container
  std::deque<std::pair<cv::Vec4i*, cv::Vec4i*>> horizontals_temporary_lines_pairs;

  // Parameters
  int MaxDoorGap = boost::lexical_cast<double>(parameters()["MaxDoorGap"]);
  int HorizontalBoxThreshold =
      boost::lexical_cast<double>(parameters()["HorizontalBoxThreshold"]);
  int VerticalBoxThreshold =
      boost::lexical_cast<double>(parameters()["VerticalBoxThreshold"]);

  if (parameters()["IsOnCUDA"] == "true") {

  } else {
    // Lines filtration with door gap parameter
    for (unsigned int i = 0; i < horizontals_lines->size() - 1; i++) {
      for (unsigned int j = i + 1; j < horizontals_lines->size(); j++) {
        if (DistanceBetweenLines((*horizontals_lines)[i],(*horizontals_lines)[j]) < MaxDoorGap) {
           horizontals_temporary_lines_pairs.push_back(
               std::pair<cv::Vec4i*, cv::Vec4i*>(&(*horizontals_lines)[i], &(*horizontals_lines)[j]));  // Adding valid line pairs to horizontal temporary lines container
        }
      }
    }

    for(auto &horizontal_temporary_line_pair : horizontals_temporary_lines_pairs) {
      for(auto &vertical_line_pair : *verticals_lines_pairs) {
        if(IsInBox(horizontal_temporary_line_pair, vertical_line_pair,
                  HorizontalBoxThreshold, VerticalBoxThreshold)) {
          // TODO (Felix) Gap found do something

          //Generate a random color
          CvRNG rng;
          int ramdomInt = cvRandInt(&rng);
          CvScalar randomColor = CV_RGB(ramdomInt&255, (ramdomInt>>8)&255, (ramdomInt>>16)&255);
          //Draw lines on screen
          cv::line(*screenImg,
                   cv::Point((*horizontal_temporary_line_pair.first)[X1],(*horizontal_temporary_line_pair.first)[Y1]),
                   cv::Point((*horizontal_temporary_line_pair.second)[X2],(*horizontal_temporary_line_pair.second)[Y2]),
                   randomColor, 1, 8, 0);
          cv::line(*screenImg,
                   cv::Point((*vertical_line_pair.first)[X1],(*vertical_line_pair.first)[Y1]),
                   cv::Point((*vertical_line_pair.first)[X2],(*vertical_line_pair.first)[Y2]),
                   randomColor, 1, 8, 0);

        }
      }
    }
  }
  //std::shared_ptr<Data> data = new std::shared_ptr<Data>(screenImg);
  output_data.set_data(screenImg);
  return output_data;
}

int DoorGapNode::DistanceBetweenLines(cv::Vec4i const& line1,cv::Vec4i const& line2) {
  int x0 = line1[X1], y0 = line1[Y1];
  int x1 = line2[X1], y1 = line2[Y1];
  int x2 = line2[X2], y2 = line2[Y2];
  int result = abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / sqrt(pow(x2 - x1,2) + pow(y2 - y1,2));

  return result;
}

bool DoorGapNode::IsInBox(std::pair<cv::Vec4i*, cv::Vec4i*> const horizontal_line_pair,
                          std::pair<cv::Vec4i*, cv::Vec4i*>const vertical_line_pair,
                          int const& HorizontalBoxThreshold,
                          int const& VerticalBoxThreshold) {

  bool isInHorizontalBox = false;
  bool isInBox = false;

  // Filtering vertical line in left box
  isInHorizontalBox=(LineHorizontalBoxValue((*horizontal_line_pair.first)[X1],
                                            (*horizontal_line_pair.first)[Y1],
                                            (*vertical_line_pair.first)[X2],
                                            (*vertical_line_pair.first)[Y2]) < HorizontalBoxThreshold);

  // Filtering vertical line in right box
  isInHorizontalBox =(LineHorizontalBoxValue((*horizontal_line_pair.first)[X2],
                                             (*horizontal_line_pair.first)[Y2],
                                             (*vertical_line_pair.second)[X2],
                                             (*vertical_line_pair.second)[Y2]) < HorizontalBoxThreshold);

  if(isInHorizontalBox) {
    isInBox = (LineVerticalBoxValue((*horizontal_line_pair.first)[Y1],(*vertical_line_pair.first)[Y2]) < VerticalBoxThreshold) ||
              (LineVerticalBoxValue((*horizontal_line_pair.first)[Y2],(*vertical_line_pair.second)[Y2]) < VerticalBoxThreshold);
  }
  return isInBox;
}

int DoorGapNode::LineHorizontalBoxValue(int x1, int y1, int x2, int y2) {
  return sqrt(pow(x1 - x2,2) + pow(y1 - y2,2));
}

int DoorGapNode::LineVerticalBoxValue(int y1, int y2) {
  return abs(y1-y2);
}
