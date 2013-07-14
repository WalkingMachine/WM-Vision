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

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_FILTRED_VERTICAL_LINE_PAIRS_NODE_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_FILTRED_VERTICAL_LINE_PAIRS_NODE_H_

#include "../vision_node.h"
#include "../data.h"
#include "../vision_node_factory.h"

class FiltredVerticalLinePairsNode: public VisionNode {
 public:
  Data Function(InputData input_data);

 private:
  REGISTER_DEC_TYPE(FiltredVerticalLinePairsNode);
};

REGISTER_DEF_TYPE(FiltredVerticalLinePairsNode);

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_FILTRED_VERTICAL_LINE_PAIRS_NODE_H_
