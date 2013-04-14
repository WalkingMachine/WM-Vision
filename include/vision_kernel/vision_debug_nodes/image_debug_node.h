/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 01/15/2013
 *
 * Programmer: Keaven Martin
 *
 * Description: Connect with openni_camera and
 *              output camera data or information
 *
 */

#ifndef WM_VISION_INCLUDE_WM_VISIONKERNEL_IMAGE_DEBUG_NODE_H_
#define WM_VISION_INCLUDE_WM_VISIONKERNEL_IMAGE_DEBUG_NODE_H_

  #include <ros/ros.h>

  #include "../vision_debug_node.h"
  #include "../data.h"
  #include "../vision_node_factory.h"

  class ImageDebugNode : public VisionDebugNode {
    public:
      ImageDebugNode() {};
      virtual ~ImageDebugNode();
      Data Function(std::map<std::string, Data> *input_data);
      void Init();

    private:
      ros::Publisher publisher_;
      REGISTER_DEC_TYPE(ImageDebugNode);
  };

  REGISTER_DEF_TYPE(ImageDebugNode);

#endif  // WM_VISION_INCLUDE_WM_VISIONKERNEL_IMAGE_DEBUG_NODE_H_
