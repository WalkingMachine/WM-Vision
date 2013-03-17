/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 12/07/2012
 *
 * Programmer: Keaven Martin
 *
 * Description: Grayscale vision filter (Temporary node)
 *
 */

#ifndef WM_VISION_INCLUDE_WM_VISIONKERNEL_GRAYSCALE_NODE_H_
#define WM_VISION_INCLUDE_WM_VISIONKERNEL_GRAYSCALE_NODE_H_

  #include <string>
  #include <map>

  #include "../include/wm_visionKernel/vision_node.h"
  #include "../include/wm_visionKernel/data.h"
  #include "../include/wm_visionKernel/vision_node_factory.h"

  class GrayscaleNode: public VisionNode {
    public:
      Data Function(std::map<std::string, Data> *input_data);

    private:
      REGISTER_DEC_TYPE(GrayscaleNode);
      int GrayscaleTypeStringToInt(std::string threshold_type);
  };

  REGISTER_DEF_TYPE(GrayscaleNode);

#endif  // WM_VISION_INCLUDE_WM_VISIONKERNEL_GRAYSCALE_NODE_H_
