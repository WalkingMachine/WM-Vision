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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <boost/lexical_cast.hpp>


Data DoorGapNode::Function(InputData input_data) {
  // Inputs
  std::shared_ptr<cv::vector<cv::Vec4i>> horizontals_lines =
      input_data->at("horizontalsLinesVector")->DataWithValidation<cv::vector<cv::Vec4i>>();
  std::shared_ptr<std::deque<std::pair<cv::Vec4i, cv::Vec4i>>> verticals_lines_pairs =
      input_data->at("verticalsLinesDeque")->DataWithValidation<std::deque<std::pair<cv::Vec4i, cv::Vec4i>>>();

  // Outputs
  Data output_data;

  // Black screen matrice
  std::shared_ptr<cv::Mat> screenImg(new cv::Mat(cv::Size(640,480),CV_8UC3));
  screenImg->setTo(0);

  // Temporary lines container
  std::deque<std::pair<cv::Vec4i*, cv::Vec4i*>> horizontals_temporary_lines_pairs;

  // Parameters
  int MaxDoorGap = boost::lexical_cast<int>(parameters()["MaxDoorGap"]);
  int HorizontalBoxThreshold =
      boost::lexical_cast<int>(parameters()["HorizontalBoxThreshold"]);
  int VerticalBoxThreshold =
      boost::lexical_cast<int>(parameters()["VericalBoxThreshold"]);

  if (parameters()["IsOnCUDA"] == "true") {

  } else {
    int *lowest_y_first_line, *bigest_y_first_line,
        *lowest_y_second_line, *bigest_y_second_line;

    // Lines filtration with door gap parameter
    for (unsigned int i = 0; i < horizontals_lines->size() - 1; i++) {
      for (unsigned int j = i + 1; j < horizontals_lines->size(); j++) {
        if (DistanceBetweenLines((*horizontals_lines)[i], (*horizontals_lines)[j]) < MaxDoorGap) {
          // Which points of the first line has lowest and bigest Y value
          if ((*horizontals_lines)[i][Y1] < (*horizontals_lines)[i][Y2]) {
            lowest_y_first_line = &(*horizontals_lines)[i][Y1];
            bigest_y_first_line = &(*horizontals_lines)[i][Y2];
          } else {
            lowest_y_first_line = &(*horizontals_lines)[i][Y2];
            bigest_y_first_line = &(*horizontals_lines)[i][Y1];
          }

          // Which points of the second line has lowest and bigest Y value
          if ((*horizontals_lines)[j][Y1] < (*horizontals_lines)[j][Y2]) {
            lowest_y_second_line = &(*horizontals_lines)[j][Y1];
            bigest_y_second_line = &(*horizontals_lines)[j][Y2];
          } else {
            lowest_y_second_line = &(*horizontals_lines)[j][Y2];
            bigest_y_second_line = &(*horizontals_lines)[j][Y1];
          }

          if (!VisibleCrossLine(*lowest_y_first_line, *bigest_y_first_line,
                                *lowest_y_second_line, *bigest_y_second_line)) {
            // Put line with lowest Y value in first in horizontal lines container
            if (*lowest_y_first_line < *lowest_y_second_line) {
              horizontals_temporary_lines_pairs.push_back(
                  std::pair<cv::Vec4i*, cv::Vec4i*>(&(*horizontals_lines)[i],
                                                    &(*horizontals_lines)[j]));
            } else {
              horizontals_temporary_lines_pairs.push_back(
                  std::pair<cv::Vec4i*, cv::Vec4i*>(&(*horizontals_lines)[j],
                                                    &(*horizontals_lines)[i]));
            }
          }
        }
      }
    }

    for (auto &horizontal_temporary_line_pair : horizontals_temporary_lines_pairs) {
      for (auto &vertical_line_pair : *verticals_lines_pairs) {
        if (IsInBox(horizontal_temporary_line_pair, vertical_line_pair,
                  HorizontalBoxThreshold, VerticalBoxThreshold)) {
          // TODO (Felix) Gap found do something

          // Generate a random color
          CvRNG rng;
          int ramdomInt = cvRandInt(&rng);
          CvScalar randomColor = CV_RGB(ramdomInt&255,
                                        (ramdomInt>>8)&255,
                                        (ramdomInt>>16)&255);
          // Draw lines on screen
          cv::line(*screenImg,
                   cv::Point((*horizontal_temporary_line_pair.first)[X1],
                             (*horizontal_temporary_line_pair.first)[Y1]),
                   cv::Point((*horizontal_temporary_line_pair.first)[X2],
                             (*horizontal_temporary_line_pair.first)[Y2]),
                   randomColor, 1, 8, 0);
          cv::line(*screenImg,
                   cv::Point((*horizontal_temporary_line_pair.second)[X1],
                             (*horizontal_temporary_line_pair.second)[Y1]),
                   cv::Point((*horizontal_temporary_line_pair.second)[X2],
                             (*horizontal_temporary_line_pair.second)[Y2]),
                   randomColor, 1, 8, 0);
          cv::line(*screenImg,
                   cv::Point(vertical_line_pair.first[X1],
                             vertical_line_pair.first[Y1]),
                   cv::Point(vertical_line_pair.first[X2],
                             vertical_line_pair.first[Y2]),
                   randomColor, 1, 8, 0);
          cv::line(*screenImg,
                   cv::Point(vertical_line_pair.second[X1],
                             vertical_line_pair.second[Y1]),
                   cv::Point(vertical_line_pair.second[X2],
                             vertical_line_pair.second[Y2]),
                   randomColor, 1, 8, 0);
        }
      }
    }
  }

  output_data.set_data(screenImg);
  return output_data;
}

int DoorGapNode::DistanceBetweenLines(cv::Vec4i const& line1,
                                      cv::Vec4i const& line2) {
  int x0 = line1[X1], y0 = line1[Y1];
  int x1 = line2[X1], y1 = line2[Y1];
  int x2 = line2[X2], y2 = line2[Y2];

  int result = abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1))
               / sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));

  return result;
}

bool DoorGapNode::IsInBox(
    const std::pair<cv::Vec4i*, cv::Vec4i*> &horizontal_line_pair,
    const std::pair<cv::Vec4i, cv::Vec4i> &vertical_line_pair,
    const int &horizontal_box_threshold,
    const int &vertical_box_threshold) {
  bool isInHorizontalBox = false;
  bool isInBox = false;

  int *lowest_x_first_line, *bigest_x_first_line;

  // Which points of the first line has lowest and bigest X value
  if ((*horizontal_line_pair.first)[X1] < (*horizontal_line_pair.first)[X2]) {
    lowest_x_first_line = &(*horizontal_line_pair.first)[X1];
    bigest_x_first_line = &(*horizontal_line_pair.first)[X2];
  } else {
    lowest_x_first_line = &(*horizontal_line_pair.first)[X2];
    bigest_x_first_line = &(*horizontal_line_pair.first)[X1];
  }

  // Filtering vertical line in left box
  isInHorizontalBox = LineHorizontalBoxValue(*lowest_x_first_line,
                                             vertical_line_pair.first[X2],
                                             horizontal_box_threshold);

  // Filtering vertical line in right box
  isInHorizontalBox = LineHorizontalBoxValue(*bigest_x_first_line,
                                             vertical_line_pair.second[X2],
                                             horizontal_box_threshold);

  if(isInHorizontalBox) {
    int *lowest_y_first_line, *bigest_y_first_line;

    // Which points of the first line has lowest and bigest Y value
    if ((*horizontal_line_pair.first)[Y1] < (*horizontal_line_pair.first)[Y2]) {
      lowest_y_first_line = &(*horizontal_line_pair.first)[Y1];
      bigest_y_first_line = &(*horizontal_line_pair.first)[Y2];
    } else {
      lowest_y_first_line = &(*horizontal_line_pair.first)[Y2];
      bigest_y_first_line = &(*horizontal_line_pair.first)[Y1];
    }

    isInBox = LineHorizontalBoxValue(*lowest_y_first_line,
                                     vertical_line_pair.first[Y2],
                                     vertical_box_threshold);

    isInBox = LineHorizontalBoxValue(*bigest_y_first_line,
                                     vertical_line_pair.second[Y2],
                                     vertical_box_threshold);
  }

  return isInBox;
}

bool DoorGapNode::LineHorizontalBoxValue(int x1, int x2, int threshhold) {
  return (abs(x2 - x1) < threshhold);
}

bool DoorGapNode ::VisibleCrossLine(
    const int lowest_y_first_line,
    const int bigest_y_first_line,
    const int lowest_y_second_line,
    const int bigest_y_second_line) {
  return ((lowest_y_first_line < lowest_y_second_line)
      && (bigest_y_first_line > bigest_y_second_line))
      || ((lowest_y_second_line < lowest_y_first_line)
      && (bigest_y_second_line > bigest_y_first_line));
}
