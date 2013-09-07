/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 14/05/2013
 *
 * Programmer: Keaven Martin
 *
 * Description:
 *
 */

#include "../../../include/vision_kernel/vision_nodes/filtred_vertical_line_pairs_node.h"

#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <boost/lexical_cast.hpp>
#include <deque>

enum {X1, Y1, X2, Y2};

Data FiltredVerticalLinePairsNode::Function(InputData input_data) {
  //Inputs
  std::shared_ptr<cv::vector<cv::Vec4i>> lines =
      input_data->at("input")->DataWithValidation<cv::vector<cv::Vec4i>>();

  //Outputs
  Data output_data;
  std::shared_ptr<std::deque<std::pair<cv::Vec4i, cv::Vec4i>>>
      filtred_line_pairs(new std::deque<std::pair<cv::Vec4i, cv::Vec4i>>);

  //Parameters
  int minimum_lines_distance =
      boost::lexical_cast<int>(parameters()["MinimumLinesDistance"]);

  if (parameters()["IsOnCUDA"] == "true") {

  } else {
	  int *lowest_X_first_line_pointer, *bigest_X_first_line_pointer,
	      *lowest_X_second_line_pointer, *bigest_X_second_line_pointer,
        *lowest_Y_first_line_pointer, *bigest_Y_first_line_pointer,
        *lowest_Y_second_line_pointer, *bigest_Y_second_line_pointer;

	  if (!lines->empty()) {
      // Improved iteration between pairs of line
      for (unsigned int i = 0; i < lines->size() - 1; i++) {
        for (unsigned int j = i + 1; j < lines->size(); j++) {
          // Which points of the first line has lowest and bigest Y value
          if ((*lines)[i][Y1] < (*lines)[i][Y2]) {
            lowest_Y_first_line_pointer = &(*lines)[i][Y1];
            bigest_Y_first_line_pointer = &(*lines)[i][Y2];
          } else {
            lowest_Y_first_line_pointer = &(*lines)[i][Y2];
            bigest_Y_first_line_pointer = &(*lines)[i][Y1];
          }

          // Which points of the second line has lowest and bigest Y value
          if ((*lines)[j][Y1] < (*lines)[j][Y2]) {
            lowest_Y_second_line_pointer = &(*lines)[j][Y1];
            bigest_Y_second_line_pointer = &(*lines)[j][Y2];
          } else {
            lowest_Y_second_line_pointer = &(*lines)[j][Y2];
            bigest_Y_second_line_pointer = &(*lines)[j][Y1];
          }

          if ((*lowest_Y_first_line_pointer < *bigest_Y_second_line_pointer) &&
              (*lowest_Y_second_line_pointer < *bigest_Y_first_line_pointer)) {  // One line isn't hover the other
            // Which points of the first line has lowest and bigest X value
            if ((*lines)[i][X1] < (*lines)[i][X2]) {
              lowest_X_first_line_pointer = &(*lines)[i][X1];
              bigest_X_first_line_pointer = &(*lines)[i][X2];
            } else {
              lowest_X_first_line_pointer = &(*lines)[i][X2];
              bigest_X_first_line_pointer = &(*lines)[i][X1];
            }

            // Which points of the second line has lowest and bigest X value
            if ((*lines)[j][X1] < (*lines)[j][X2]) {
              lowest_X_second_line_pointer = &(*lines)[j][X1];
              bigest_X_second_line_pointer = &(*lines)[j][X2];
            } else {
              lowest_X_second_line_pointer = &(*lines)[j][X2];
              bigest_X_second_line_pointer = &(*lines)[j][X1];
            }

            if (*lowest_X_second_line_pointer - *bigest_X_first_line_pointer >
                minimum_lines_distance) {
              cv::Vec4i first_line(*(lowest_Y_first_line_pointer - 1),
                                   *lowest_Y_first_line_pointer,
                                   *(bigest_Y_first_line_pointer - 1),
                                   *bigest_Y_first_line_pointer);
              cv::Vec4i second_line(*(lowest_Y_second_line_pointer - 1),
                                    *lowest_Y_second_line_pointer,
                                    *(bigest_Y_second_line_pointer - 1),
                                    *bigest_Y_second_line_pointer);

              if (*lowest_X_first_line_pointer < *lowest_X_second_line_pointer) {
                filtred_line_pairs->push_back(std::pair<cv::Vec4i, cv::Vec4i>(first_line, second_line)); // Add filtred pair of line
              } else {
                filtred_line_pairs->push_back(std::pair<cv::Vec4i, cv::Vec4i>(second_line, first_line)); // Add filtred pair of line
              }
            }
          }
        }
      }
	  }

	  output_data.set_data(filtred_line_pairs);
  }

  return output_data;
}
