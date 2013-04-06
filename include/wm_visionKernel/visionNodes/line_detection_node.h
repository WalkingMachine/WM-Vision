/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 03/18/2013
 *
 * Programmer: Keaven Martin
 *
 * Description: Image strait line detection
 *
 */

#ifndef WM_VISION_INCLUDE_WM_VISIONKERNEL_LINE__DETECTION_NODE_H_
#define WM_VISION_INCLUDE_WM_VISIONKERNEL_LINE__DETECTION_NODE_H_

  #include <string>
  #include <map>

  #include "../include/wm_visionKernel/vision_node.h"
  #include "../include/wm_visionKernel/data.h"
  #include "../include/wm_visionKernel/vision_node_factory.h"

  class LineDetectionNode: public VisionNode {
    public:
      Data Function(std::map<std::string, Data> *input_data);

    private:
      REGISTER_DEC_TYPE(LineDetectionNode);
  };

  REGISTER_DEF_TYPE(LineDetectionNode);

#endif  // WM_VISION_INCLUDE_WM_VISIONKERNEL_LINE__DETECTION_NODE_H_