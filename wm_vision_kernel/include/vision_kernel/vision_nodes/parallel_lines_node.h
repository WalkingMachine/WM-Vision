/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 06/04/2013
 *
 * Programmer: Keaven Martin
 *
 * Description:
 *
 */

#ifndef WM_VISION_INCLUDE_WM_VISIONKERNEL_PARALLEL_LINES_NODE_H_
#define WM_VISION_INCLUDE_WM_VISIONKERNEL_PARALLEL_LINES_NODE_H_

  #include <string>
  #include <map>

  #include "../vision_node.h"
  #include "../data.h"
  #include "../vision_node_factory.h"

  class ParallelLinesNode: public VisionNode {
    public:
      Data Function(InputData input_data);

    private:
      REGISTER_DEC_TYPE(ParallelLinesNode);
  };

  REGISTER_DEF_TYPE(ParallelLinesNode);

#endif  // WM_VISION_INCLUDE_WM_VISIONKERNEL_PARALLEL_LINES_NODE_H_
