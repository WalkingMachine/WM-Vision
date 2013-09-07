/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 09/07/2013
 *
 * Programmer: Keaven Martin
 *
 * Description:
 *
 */

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_TEXTURE_BOTTOM_OF_DOOR_NODE_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_TEXTURE_BOTTOM_OF_DOOR_NODE_H_

#include "../vision_node.h"
#include "../data.h"
#include "../vision_node_factory.h"

#include <string>

class TextureBottomOfDoorNode: public VisionNode {
 public:
  TextureBottomOfDoorNode(){};
  ~TextureBottomOfDoorNode(){};
  Data Function(InputData input_data);

 private:
  typedef enum {
    LEFT_VS_CENTER,
    RIGHT_VS_CENTER,
    ALL
  } ValidationType;
  ValidationType StringToValidationType(std::string validationType);
  REGISTER_DEC_TYPE(TextureBottomOfDoorNode);
};

REGISTER_DEF_TYPE(TextureBottomOfDoorNode);

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_TEXTURE_BOTTOM_OF_DOOR_NODE_H_
