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

#include <rqt_vision_tools/vision_parser.h>

#include <boost/property_tree/exceptions.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/exceptions.hpp>
#include <boost/foreach.hpp>


VisionParser::CFVTPathMap VisionParser::ParseCatalogCFVT(std::string file_path) {
  typedef boost::property_tree::ptree ptree;

  CFVTPathMap cfvt_path_map;

  try {
    ptree property_tree;
    boost::property_tree::read_info(file_path, property_tree);
    std::string task_name;
    std::string object_name;

    for (const ptree::value_type &tasktype_iteration : property_tree.get_child("")) {
      std::map<std::string, std::string> object;
      for (const ptree::value_type &objectid_iteration : tasktype_iteration.second) {
        task_name = tasktype_iteration.first.data();
        object_name = objectid_iteration.first.data();

	object.insert(std::pair<std::string, std::string>(object_name, objectid_iteration.second.data()));
        /*path_map[std::make_pair(task_name, object_name)] =
            objectid_iteration.second.data();*/
      }
      
      cfvt_path_map.insert(std::pair<std::string, std::map<std::string, std::string>>(task_name, object));
    }

  } catch(const std::exception& e) {
      throw e;
  }

  return cfvt_path_map;
}

VisionParser::VisionTree VisionParser::ParseVisionTree(std::string file_path) {
  typedef boost::property_tree::ptree ptree;
  VisionTree vision_tree;
  Node node;

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
      node.type = node_iteration.first.data();
      node.dependences = dependences;
      node.parameters = parameters;
      node.debug_node = debug_node_name;
      vision_tree.insert(std::pair<std::string, Node>(id, node));
    }

    // TODO(Keaven Martin) Valid dependence
  }
  catch(const std::exception &exception) {
    throw;
  }
  return vision_tree;
}

void VisionParser::SaveVisionTree(std::string file_path, VisionTree vision_tree) {
	boost::property_tree::ptree property_tree;

	for(auto node: vision_tree) {
		property_tree.add(node.second.type + ".id", node.first);
		property_tree.add(node.second.type + ".debug", node.second.debug_node);

		for(auto dependence: node.second.dependences)
			property_tree.add(node.second.type + ".dependence." + dependence.first, dependence.second);

		for(auto parameter: node.second.parameters)
			property_tree.add(node.second.type + ".parameter." + parameter.first, parameter.second);
	}

	boost::property_tree::write_info(file_path, property_tree);
}
