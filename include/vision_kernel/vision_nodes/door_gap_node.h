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

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_DOOR_GAP_NODE_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_DOOR_GAP_NODE_H_

#include "../vision_node.h"
#include "../data.h"
#include "../vision_node_factory.h"

#include <opencv2/opencv.hpp>

enum {X1, Y1, X2, Y2};

class DoorGapNode: public VisionNode {
 public:
  Data Function(InputData input_data);

 private:
  int DistanceBetweenLines(cv::Vec4i const& line1,cv::Vec4i const& line2);
  bool IsInBox(const std::pair<cv::Vec4i*, cv::Vec4i*> &horizontal_line_pair,
               const std::pair<cv::Vec4i, cv::Vec4i> &vertical_line_pair,
               const int &horizontal_box_threshold,
               const int &vertical_box_threshold);
  bool LineHorizontalBoxValue(int x1, int x2, int threshhold);
  int LineVerticalBoxValue(int y1, int y2);
  bool VisibleCrossLine(const int lowest_y_first_line,
                        const int bigest_y_first_line,
                        const int lowest_y_second_line,
                        const int bigest_y_second_line);
  REGISTER_DEC_TYPE(DoorGapNode);
};

REGISTER_DEF_TYPE(DoorGapNode);

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODES_DOOR_GAP_NODE_H_
