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

#include "../../include/vision_kernel/vision_interface.h"

#include <ros/ros.h>

#include "../../include/vision_kernel/vision_parser.h"

VisionInterface::VisionInterface() {
  // parse catalog.info for the cfvt paths, store the info in path_map
  try {
    VisionParser::ParseCatalogCFVT(path_map);
  } catch(const std::exception& e) {
      throw e;
  }
}

bool VisionInterface::CallbackTree(
    wm_vision::vision_interface_tree::Request& request,
    wm_vision::vision_interface_tree::Response& response) {

  float frequency = (request.frequency == 0 ? 30 : request.frequency);

  std::string tree_name;

  // store request paramter into a std::pair
  std::pair<std::string, std::string> key(request.task_name,
                                          request.object_name);

  // exit if the requested task isn't in catalog.info
  auto path_iterator = path_map.find(key);
  if(path_iterator == path_map.end()) {
    return false;
  } else {
    tree_name = path_iterator->second;
  }

	// Look within the tasks collection to see if the task already exists
	auto tree_iterator = tree_container_.find(key);
	if(tree_iterator != tree_container_.end()) {
    if(request.action == "stop") {
      tree_iterator->second->Stop();
      tree_container_.erase(tree_iterator);
    } else if(request.action == "start") {
      //task_iterator->second->Start();
    } else if(request.action == "set_frequency") {
      tree_iterator->second->set_wanted_frequency(frequency);
    } else {
      response.topic_name = "invalid Action name";
    }
  } else {
    CreateTree(request.task_name, request.object_name, tree_name, frequency);
    response.topic_name = tree_name;
  }
	return true;
}

void VisionInterface::CreateTree(const std::string &task_name,
                                 const std::string &object_name,
                                 const std::string &tree_name,
                                 const float &frequency) {
  std::shared_ptr<VisionTree> vision_tree(new VisionTree(tree_name, frequency));
  ros::NodeHandle node_handle;
  std::string cfvt_path;

  node_handle.getParamCached("cfvt_path",cfvt_path);  // Get rosparam cfvt_path

  try {
    VisionParser::ParseVisionTree(std::string(cfvt_path + tree_name + ".cfvt"),
                                  *vision_tree);
  } catch(const std::exception& e) {
    ROS_INFO("%s", e.what());
  }

  std::pair<std::string,std::string> key(task_name, object_name);
  tree_container_[key] = vision_tree;
  vision_tree->Start();
}
