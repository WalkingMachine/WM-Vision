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
 * Description: VisionCfvfParser is used to parse a file which holds the path
 *              to the flows to execute.
 *
 */

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_PARSER_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_PARSER_H_

#include <string>
#include <map>

#include "vision_flow.h"

namespace VisionParser{
  /**
   *  Fills a map with a pair of task_id and object_id to a path that points
   *  to the corresponding configuration file to create the flow
   * @param path_map A reference to the map that needs to be filled
   */
  void ParseCatalogCFVF(
      std::map<std::pair<std::string,std::string>, std::string> &path_map);

  /**
   * Parse a .cfvf config file to create a visaion flow
   * @param file_path
   * @param flow The flow to create
   */
  void ParseVisionFlow(std::string file_path, VisionFlow &flow);

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

  class CatalogPathException : public std::exception {
    virtual const char* what() const throw() {
        return "Catalog path not set in the parameters";
      }
  };

  class InvalidCatalogException : public std::exception {
    virtual const char* what() const throw() {
        return "Invalid syntax in catalog configuration file";
      }
  };

  class InvalidCfvfException : public std::exception {
    std::string file;
    virtual const char* what() const throw() {
        return ("Invalid syntax in the cfvf (" + file + ")").c_str();
      }
   public:
    InvalidCfvfException(std::string file) : file(file) {}
  };
}
#endif  // WM_VISION_INCLUDE_VISION_KERNEL_VISION_PARSER_H_
