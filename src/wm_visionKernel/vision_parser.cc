/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 02/24/2013
 *
 * Programmer: Julien Côté, Keaven Martin
 *
 * Description: VisionCfvtParser is used to parse a file which holds the path
 *              to the trees to execute.
 *
 */

#include "../include/wm_visionKernel/vision_parser.h"

#include <boost/property_tree/exceptions.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/exceptions.hpp>
#include <boost/foreach.hpp>


void VisionParser::ParseCatalogCFVT(
    std::map<std::pair<std::string,std::string>, std::string > &path_map){
  ros::NodeHandle node_handle;
  std::string file_path;

  node_handle.getParamCached("catalog_path",file_path);

  typedef boost::property_tree::ptree ptree;

  try {
    ptree property_tree;
    boost::property_tree::read_info(file_path, property_tree);
    std::string task_name;
    std::string object_name;

    for (const ptree::value_type &tasktype_iteration : property_tree.get_child("")) {
      for (const ptree::value_type &objectid_iteration : tasktype_iteration.second) {
        task_name = tasktype_iteration.first.data();
        object_name = objectid_iteration.first.data();

        path_map[std::make_pair(task_name, object_name)] =
            objectid_iteration.second.data();
      }
    }

  } catch(const std::exception& e) {
      throw e;
  }
}

void VisionParser::ParseVisionTree(std::string file_path, VisionTree *tree) {
  typedef boost::property_tree::ptree ptree;

  try {
    boost::property_tree::ptree propertyTree;
    boost::property_tree::read_info(file_path, propertyTree);

    for (const ptree::value_type &node_iteration : propertyTree.get_child("")) {
      std::string id;
      std::string debug_node_name = "";
      std::map<std::string, std::string> dependences;
      std::map<std::string, std::string> parameters;

      for (const ptree::value_type &parameter_iteration : node_iteration.second) {
        if (!strcmp(parameter_iteration.first.data(), "id")) {
          id = parameter_iteration.second.data();
        } else if (!strcmp(parameter_iteration.first.data(), "debug")) {
          debug_node_name = parameter_iteration.second.data();
        } else if (!strcmp(parameter_iteration.first.data(), "dependence")) {
          for (const ptree::value_type &value : parameter_iteration.second) {
            dependences.insert(std::pair<std::string, std::string>(
                                   value.first.data(), value.second.data()));
          }
        } else if (!strcmp(parameter_iteration.first.data(), "parameter")) {
          for (const ptree::value_type &value : parameter_iteration.second) {
            parameters.insert(std::pair<std::string, std::string>(
                                  value.first.data(), value.second.data()));
          }
        } else {
          throw NodeParameterException();
        }
      }

      if (!tree->AddNode(node_iteration.first.data(), id, dependences, parameters, debug_node_name)) {
        throw NodeValidationException();
      }
    }

    if (!tree->IsValid())
      throw NodeValidationException();

    // TODO(Keaven Martin) Valid dependence
  }
  catch(const std::exception &exception) {
    throw;
  }
}
