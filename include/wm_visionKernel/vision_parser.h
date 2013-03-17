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

#include "wm_visionKernel/vision_tree.h"

namespace VisionParser{
  /**
   *  Fills a map with a pair of task_id and object_id to a path that points
   *  to the corresponding configuration file to create the tree
   * @param path_map A reference to the map that needs to be filled
   */
  void ParseCatalogCFVT(
      std::map<std::pair<std::string,std::string>, std::string > &path_map);

  void ParseVisionTree(std::string file_path, VisionTree *tree);

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
}
#endif  // VISION_PARSER_H_
