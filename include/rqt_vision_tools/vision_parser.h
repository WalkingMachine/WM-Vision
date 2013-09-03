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

#ifndef VISION_PARSER_H_
#define VISION_PARSER_H_

#include <string>
#include <map>
#include <set>

namespace VisionParser{
  /**
   *  Fills a map with a pair of task_id and object_id to a path that points
   *  to the corresponding configuration file to create the tree
   * @param path_map A reference to the map that needs to be filled
   */
    typedef std::map<std::string, std::map<std::string, std::string>> CFVTPathMap;  //First: action; Second: object + path

  class Node {
    public:
      std::string type;
      std::string debug_node;
      std::map<std::string, std::string> dependences;
      std::map<std::string, std::string> parameters;
  };

  typedef std::map<std::string, Node> VisionTree;  //First: node id; Second: node

  class NodeParameterException : public std::exception {
    virtual const char* what() const throw() {
        return "Invalid node parameter";
      }
  };

  class NodeValidationException : public std::exception {
    virtual const char* what() const throw() {
        return "Invalid node creation";
      }
  };

  CFVTPathMap ParseCatalogCFVT(std::string file_path);

  VisionTree ParseVisionTree(std::string file_path);

  void SaveVisionTree(std::string file_path, VisionTree vision_tree);
}
#endif  // VISION_PARSER_H_
