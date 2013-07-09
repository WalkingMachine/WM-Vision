/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 15/05/2013
 *
 * Programmer: Keaven Martin
 *
 * Description:
 *
 */

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_DIFFERENT_COLOR_BETWEEN_DOOR_AND_WALL_NODE_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_DIFFERENT_COLOR_BETWEEN_DOOR_AND_WALL_NODE_H_

#include "../vision_node.h"
#include "../data.h"
#include "../vision_node_factory.h"

#include <string>

class DifferentColorBetwenDoorAndWallNode: public VisionNode {
 public:
  DifferentColorBetwenDoorAndWallNode(){};
  ~DifferentColorBetwenDoorAndWallNode(){};
  Data Function(InputData input_data);

 private:
  typedef enum {
    LEFT_VS_CENTER,
    RIGHT_VS_CENTER,
    ALL
  } ValidationType;
  double DeltaE(double L1, double a1, double b1,
                double L2, double a2, double b2);
  ValidationType StringToValidationType(std::string validationType);
  REGISTER_DEC_TYPE(DifferentColorBetwenDoorAndWallNode);
};

REGISTER_DEF_TYPE(DifferentColorBetwenDoorAndWallNode);

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_DIFFERENT_COLOR_BETWEEN_DOOR_AND_WALL_NODE_H_
