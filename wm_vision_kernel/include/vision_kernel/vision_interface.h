/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: vision_kernel
 *
 * Creation date: 02/07/2013
 *
 * Programmer: Julien Côté
 *
 * Description:
 *
 */

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_INTERFACE_HPP_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_INTERFACE_HPP_

#include <map>
#include <memory>
#include <wm_vision_kernel/wm_vision_interface_flow.h>  //TODO(Julien) Verif

#include "vision_flow.h"

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
	bool CallbackFlow(wm_vision_kernel::wm_vision_interface_flow::Request& request,
	                  wm_vision_kernel::wm_vision_interface_flow::Response& response);

 private:
	// TODO(Keaven) Change std::map
	std::map<std::pair<std::string, std::string>, std::shared_ptr<VisionFlow>>
	  flow_container_;
	std::map<std::pair<std::string,std::string>, std::string> path_map_;

	/**
	 *
	 * @param id_task the task to execute. Refer to the enum for the names
	 * @param id_object The object, person or gesture to be found
	 * @param tree_name The name of the vision_tree to call
	 */
	void CreateFlow(const std::string &task_name,
	                const std::string &object_name,
	                const std::string &flow_name,
	                const float &frequency);
};

#endif  //  WM_VISION_INCLUDE_VISION_KERNEL_VISION_INTERFACE_HPP_
