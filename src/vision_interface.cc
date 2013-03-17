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
 * Description: wm_vision_interface is an interface used externally to create
 *              and manage vision_trees execution. It is composed of a ros SRV
 *              and the data is outputed via ros topics. Each tree outputs on
 *              it's own topic. The name of these topics is returned by the SRV.
 *
 */

#include "../include/vision_interface.h"
#include "../include/wm_visionKernel/vision_parser.h"

VisionInterface::VisionInterface() {
  // parse catalog.info for the cfvt paths, store the info in path_map
  try{
    VisionParser::ParseCatalogCFVT(path_map);
  } catch(const std::exception& e){
      throw e;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vision_kernel");
  ros::NodeHandle node_handle;
  VisionInterface interface;

  ros::ServiceServer service = node_handle.advertiseService(
      ros::this_node::getName(),
      &VisionInterface::CallbackTree,
      &interface);

  ROS_INFO("vision_kernel initialized");
  ros::spin();

  return 0;
}

bool VisionInterface::CallbackTree(wm_vision::vision_interface_tree::Request& request,
                                wm_vision::vision_interface_tree::Response& response){
  std::string task_name = request.task_name;
  std::string object_name = request.object_name;
  std::string action = request.action;
  float frequency = (request.frequency == 0 ? 30 : request.frequency);

  std::string tree_name;

    //  store request paramter into a std::pair
  std::pair<std::string,std::string> key(task_name, object_name);

    //  exit if the requested task isn't in catalog.info
  auto path_iterator = path_map.find(key);
  if(path_iterator == path_map.end()){
    return false;
  } else{
    tree_name = path_iterator->second;
  }

	  //  Look within the tasks collection to see if the task already exists
	auto tree_iterator = tree_container_.find(key);
	if(tree_iterator != tree_container_.end()) {
    if(action == "stop") {
      tree_iterator->second->Stop();
      tree_container_.erase(tree_iterator);
    } else if(action == "start") {
      //task_iterator->second->Start();
    } else if(action == "set_frequency") {
      tree_iterator->second->set_wanted_frequency(frequency);
    } else {
      response.topic_name = "invalid Action name";
    }
  } else {
    CreateTree(task_name, object_name, tree_name, frequency);
    response.topic_name = tree_name;
  }
	return true;
}

void VisionInterface::CreateTree(std::string task_name,
                                 std::string object_name,
                                 std::string tree_name,
                                 float frequency) {
  std::shared_ptr<VisionTree> vision_tree(new VisionTree(tree_name, frequency));
  ros::NodeHandle node_handle;
  std::string cfvt_path;

  node_handle.getParamCached("cfvt_path",cfvt_path);
  try{
    VisionParser::ParseVisionTree(std::string(cfvt_path + tree_name + ".cfvt"), vision_tree.get());
  } catch(const std::exception& e) {
    ROS_INFO("%s", e.what());
    throw e;
  }

  std::pair<std::string,std::string> key(task_name, object_name);
  tree_container_[key] = vision_tree;
  vision_tree->Start();
}
