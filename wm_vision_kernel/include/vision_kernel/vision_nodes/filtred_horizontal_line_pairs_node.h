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

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_FILTRED_HORIZONTAL_LINE_PAIRS_NODE_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_FILTRED_HORIZONTAL_LINE_PAIRS_NODE_H_

#include "../vision_node.h"
#include "../data.h"
#include "../vision_node_factory.h"

class FiltredHorizontalLinePairsNode: public VisionNode {
 public:
  Data Function(InputData input_data);

 private:
  bool FiltredHorizontalLinePairsNode::VisibleCrossLine(
      const int lowest_y_first_line,
      const int bigest_y_first_line,
      const int lowest_y_second_line,
      const int bigest_y_second_line);
  REGISTER_DEC_TYPE(FiltredHorizontalLinePairsNode);
};

REGISTER_DEF_TYPE(FiltredHorizontalLinePairsNode);

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_FILTRED_HORIZONTAL_LINE_PAIRS_NODE_H_
