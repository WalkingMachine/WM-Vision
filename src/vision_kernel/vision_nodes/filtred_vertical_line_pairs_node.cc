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
#include <deque>
#include <utility>

enum {X1, Y1, X2, Y2};

Data FiltredVerticalLinePairsNode::Function(InputData input_data) {
  std::shared_ptr<cv::vector<cv::Vec4i>> lines = input_data->at("input")->DataWithValidation<cv::vector<cv::Vec4i>>();
  Data output_data;

  std::shared_ptr<std::deque<std::pair<cv::Vec4i, cv::Vec4i>>> filtred_line_pairs(new std::deque<std::pair<cv::Vec4i, cv::Vec4i>>);

  if (parameters()["IsOnCUDA"] == "true") {

  } else {
	  int *lowest_X_first_line_pointer, *bigest_X_first_line_pointer,
	      *lowest_X_second_line_pointer, *bigest_X_second_line_pointer;

	  // Improved iteration between pairs of line
	  for (unsigned int i = 0; i < lines->size(); i++) {
		  for (unsigned int j = i + 1; j <= lines->size(); j++) {
			  // Which point of the first line has lowest value and bigest value
			  if ((*lines)[i][X1] < (*lines)[i][X2]) {
			    lowest_X_first_line_pointer = &(*lines)[i][X1];
			    bigest_X_first_line_pointer = &(*lines)[i][X2];
			  } else {
			    lowest_X_first_line_pointer = &(*lines)[i][X2];
			    bigest_X_first_line_pointer = &(*lines)[i][X1];
			  }

			  // Which point of the second line has lowest value and bigest value
			  if ((*lines)[j][X1] < (*lines)[j][X2]) {
			    lowest_X_second_line_pointer = &(*lines)[j][X1];
				bigest_X_second_line_pointer = &(*lines)[j][X2];
			  } else {
				  lowest_X_second_line_pointer = &(*lines)[j][X2];
				  bigest_X_second_line_pointer = &(*lines)[j][X1];
			  }

			  if ((*lowest_X_first_line_pointer < *bigest_X_second_line_pointer) &&
			      (*lowest_X_second_line_pointer < *bigest_X_first_line_pointer)) {
			    filtred_line_pairs->push_back(std::pair<cv::Vec4i, cv::Vec4i>((*lines)[i], (*lines)[j])); // Add filtred pair of line
			  }
		  }
	  }

	  output_data.set_data(filtred_line_pairs);
  }

  return output_data;
}
