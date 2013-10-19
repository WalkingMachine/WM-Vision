/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision_kernel
 *
 * Creation date: 02/24/2013
 *
 * Programmer: Julien Côté, Keaven Martin
 *
 * Description: The VisionParser namespace contain all parser of wm_vision_kernel
 * 				(cathalog and vision flow parser)
 *
 */

#include "../../include/vision_kernel/vision_parser.h"

#include <boost/property_tree/exceptions.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>


void VisionParser::ParseCatalogCFVF(
    std::map<std::pair<std::string,std::string>, std::string> &path_map) {
  typedef boost::property_tree::ptree ptree;

  ros::NodeHandle node_handle;
  std::string catalog_path;

  if (!node_handle.getParamCached("catalog_path",catalog_path)) {
	  throw CatalogPathException();
  }

  try {
    ptree property_tree;
    boost::property_tree::read_info(catalog_path, property_tree);
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
  } catch (const std::exception& e) {
      throw InvalidCatalogException();
  }
}

void VisionParser::ParseVisionFlow(std::string file_path, VisionFlow &flow) {
  typedef boost::property_tree::ptree ptree;

  try {
    ptree property_tree;
    boost::property_tree::read_info(file_path, property_tree);

    for (const ptree::value_type &node_iteration : property_tree.get_child("")) {
      std::string id;
      std::string debug_node_name = "";
      std::map<std::string, std::string> dependences;
      std::map<std::string, std::string> parameters;

      for (const ptree::value_type &parameter_iteration : node_iteration.second) {
        if (!strcmp(parameter_iteration.first.data(), "id")) {  // TODO(Keaven) remove strcmp
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

      if (!flow.AddNode(node_iteration.first.data(), id, dependences, parameters, debug_node_name)) {
        throw NodeValidationException();
      }
    }

    if (!flow.IsValid())
      throw NodeValidationException();

    // TODO(Keaven Martin) Valid dependence
  } catch (const std::exception &e) {
    throw InvalidCfvfException(file_path);
  }
}
