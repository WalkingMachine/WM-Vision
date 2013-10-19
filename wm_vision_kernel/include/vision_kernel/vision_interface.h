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

//ROS services includes
#include <wm_vision_kernel/wm_vision_interface_flow.h>

#include "vision_flow.h"

class VisionInterface {
 public:
  VisionInterface();
	bool CallbackFlow(wm_vision_kernel::wm_vision_interface_flow::Request& request,
	                  wm_vision_kernel::wm_vision_interface_flow::Response& response);

 private:
	// TODO(Keaven) Change std::map
	std::map<std::pair<std::string, std::string>, std::shared_ptr<VisionFlow>>
	  flow_container_;
	std::map<std::pair<std::string,std::string>, std::string> path_map_;

	void CreateFlow(const std::string &task_name,
	                const std::string &object_name,
	                const std::string &flow_name,
	                const float &frequency);

	/**
	 *
	 */
  class CfvfPathException : public std::exception {
    virtual const char* what() const throw() {
        return "CFVF path not set in the parameters";
      }
  };
};

#endif  //  WM_VISION_INCLUDE_VISION_KERNEL_VISION_INTERFACE_HPP_
