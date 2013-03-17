/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 02/07/2013
 *
 * Programmer: Julien Côté
 *
 * Description: WMVisionInterface is an interface used externally to create
 *              and manage vision_trees execution. It is composed of a ros SRV
 *              and the data is outputed via ros topics. Each tree outputs on
 *              it's own topic. The name of these topics is returned by the SRV.
 *
 */

#ifndef VISION_INTERFACE_HPP_
#define VISION_INTERFACE_HPP_

#include "ros/ros.h"
#include <map>
#include <memory>
#include <wm_vision/vision_interface_tree.h>
#include "../include/wm_visionKernel/vision_tree.h"

class VisionInterface {

public:
  /**
   * Creates the initial catalog map  holding the paths to the trees
   * configuration files
   */
  VisionInterface();
  /**
   * The method used by the ros:Service when it is called
   *
   * @param request contains the parameters as defined in visionInterface.srv
   * @param response contains the return value as defined in visionInterface.srv
   * @return  True if succeeded, False if failed and won't send the Response
   */
	bool CallbackTree(wm_vision::vision_interface_tree::Request& request,
	              wm_vision::vision_interface_tree::Response& response);

private:
	std::map<std::pair<std::string, std::string>, std::shared_ptr<VisionTree> >
	  tree_container_;
	std::map<std::pair<std::string,std::string>, std::string > path_map;

	/**
	 *
	 * @param id_task the task to execute. Refer to the enum for the names
	 * @param id_object The object, person or gesture to be found
	 * @param tree_name The name of the vision_tree to call
	 */
	void CreateTree(std::string task_name,
                  std::string object_name,
                  std::string tree_name,
                  float frequency);
};

#endif  //  VISION_INTERFACE_HPP_
