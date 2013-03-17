/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 26/02/2013
 *
 * Programmer: Keaven Martin
 *
 * Description: Output debug data (EX: via ROS Topic)
 *
 */

#ifndef WM_VISION_INCLUDE_WM_VISIONKERNEL_VISION_DEBUG_NODE_H_
#define WM_VISION_INCLUDE_WM_VISIONKERNEL_VISION_DEBUG_NODE_H_

  #include <string>
  #include <map>

  #include "../include/wm_visionKernel/vision_node.h"

  class VisionDebugNode : public VisionNode {
    public:
      // Constructor and Destructor;
      VisionDebugNode();
      virtual ~VisionDebugNode();

      void set_vision_node(VisionNode* vision_node);

      void CallbackFunction(VisionNode* vision_node, Data output_data);
      virtual Data Function(std::map<std::string, Data> *input_data) = 0;
      virtual void Init() {};

      void Thread(std::map<std::string, Data> *input_data);

    private:
      VisionNode* vision_node_;
  };

#endif  // WM_VISION_INCLUDE_WM_VISIONKERNEL_VISION_DEBUG_NODE_H_
