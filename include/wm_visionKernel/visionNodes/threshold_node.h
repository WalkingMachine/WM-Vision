/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 02/25/2013
 *
 * Programmer: Julien Côté
 *
 * Description: OpenCV threshold vision filter
 *
 */

#ifndef WM_VISION_INCLUDE_WM_VISIONKERNEL_THRESHOLD_NODE_H_
#define WM_VISION_INCLUDE_WM_VISIONKERNEL_THRESHOLD_NODE_H_

  #include <string>
  #include <map>

  #include "../include/wm_visionKernel/vision_node.h"
  #include "../include/wm_visionKernel/data.h"
  #include "../include/wm_visionKernel/vision_node_factory.h"

  class ThresholdNode: public VisionNode {
    public:
      Data Function(std::map<std::string, Data> *input_data);

    private:
      int ThresholdTypeStringToInt(std::string threshold_type);
      REGISTER_DEC_TYPE(ThresholdNode);
  };

  REGISTER_DEF_TYPE(ThresholdNode);

#endif  // WM_VISION_INCLUDE_WM_VISIONKERNEL_THRESHOLD_NODE_H_