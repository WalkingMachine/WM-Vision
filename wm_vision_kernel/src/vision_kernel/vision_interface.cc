/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision_kernel
 *
 * Creation date: 02/07/2013
 *
 * Programmer: Julien Côté
 *
 * Description: The vision interface was the interface between the wm_vision_kernel ans ROS.
 *
 */

#include "../../include/vision_kernel/vision_interface.h"

#include <ros/ros.h>

#include "../../include/vision_kernel/vision_parser.h"

#include "../../include/vision_kernel/vision_errors.h"

/**
 * Creates the initial catalog map holding the paths to the trees
 * configuration files
 */
VisionInterface::VisionInterface() {
  // Parse catalog.info for the cfvf paths, store the info in path_map
  try {
    VisionParser::ParseCatalogCFVF(path_map_);
  } catch(const std::exception& e) {
	  ROS_ERROR("Invalid syntax in catalog configuration file : %s",e.what());
  }
}

/**
 * The method used by the ros:Service when it is called
 *
 * @param request contains the parameters as defined in visionInterface.srv
 * @param response contains the return value as defined in visionInterface.srv
 * @return  True if succeeded, False if failed and won't send the Response
 */
bool VisionInterface::CallbackFlow(
    wm_vision_kernel::wm_vision_interface_flow::Request& request,
    wm_vision_kernel::wm_vision_interface_flow::Response& response) {

  float frequency = (request.frequency == 0 ? 30 : request.frequency);

  std::string flow_name;
  response.error = vision_errors::NoError;

  if (!path_map_.empty())
  {
	  // store request parameter into a std::pair
	  std::pair<std::string, std::string> key(request.task_name,
											  request.object_name);

	  // exit if the requested task isn't in catalog.info
	  auto path_iterator = path_map_.find(key);
	  if(path_iterator == path_map_.end()) {
	    return false; // TODO (m-a) : Continue and set an error
	  } else {
	    flow_name = path_iterator->second;
	  }

		// Look within the tasks collection to see if the task already exists
		auto flow_iterator = flow_container_.find(key);
		if(flow_iterator != flow_container_.end()) {
      if(request.action == "stop") {
        flow_iterator->second->Stop();
        flow_container_.erase(flow_iterator);
      } else if(request.action == "start") {
      } else if(request.action == "set_frequency") {
        flow_iterator->second->set_wanted_frequency(frequency);
      } else {
        response.topic_name = "invalid Action name";
        response.error = vision_errors::InvalidActionName;
        ROS_ERROR("Invalid action name");
      }
	  } else {
	    try {
	      CreateFlow(request.task_name, request.object_name, flow_name, frequency);
        response.topic_name = flow_name;
	    } catch (const std::exception& e) {
	      ROS_ERROR("Invalid syntax in the cfvf : %s", e.what());
	      response.error = vision_errors::FlowCreationError;
	    }
	  }
  }
  else
  {
	  response.error = vision_errors::CatalogNotLoaded;
	  ROS_ERROR("Catalog configuration file not loaded");
  }
  return true;
}

/**
 * Create a vision flow
 * @param id_task the task to execute. Refer to the enum for the names
 * @param id_object The object, person or gesture to be found
 * @param tree_name The name of the vision_tree to call
 */
void VisionInterface::CreateFlow(const std::string &task_name,
                                 const std::string &object_name,
                                 const std::string &flow_name,
                                 const float &frequency) {
  std::shared_ptr<VisionFlow> vision_flow(new VisionFlow(flow_name, frequency));
  ros::NodeHandle node_handle;
  std::string cfvf_path;

  // Get rosparam cfvf_path
  if (!node_handle.getParamCached("cfvf_path",cfvf_path))
    throw VisionInterface::CfvfPathException();

  try {
    VisionParser::ParseVisionFlow(std::string(cfvf_path + flow_name + ".cfvf"),
                                  *vision_flow);
  } catch(const std::exception& e) {
    throw;
  }

  std::pair<std::string,std::string> key(task_name, object_name);
  flow_container_[key] = vision_flow;
  vision_flow->Start();
}
