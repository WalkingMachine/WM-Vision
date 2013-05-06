/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: vision_kernel
 *
 * Creation date: 12/06/2012
 *
 * Programmer: Keaven Martin
 *
 * Description: Tree's children
 *
 */

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_H_

#include <boost/thread.hpp>
#include <string>
#include <map>
#include <memory>

#include "data.h"

// TODO(Keaven) pointer

class VisionNode : public std::enable_shared_from_this<VisionNode> {
 public:
  // Type definition
  typedef std::shared_ptr<std::map<std::string, std::shared_ptr<Data>>> InputData;
  typedef std::map<std::string, std::string> Dependences;
  typedef std::map<std::string, std::string> Parameters;

  // Constructor and Destructor
  VisionNode();
  virtual ~VisionNode();

  // Getter
  std::string id();
  std::string tree_name();
  Parameters parameters();
  Dependences dependences();

  // Setter
  void set_id(const std::string &id);
  void set_tree_name(const std::string &tree_name);
  void set_parameters(const Parameters &parameters);
  void set_dependences(const Dependences &dependences);
  void set_tree_callback_function_(
            boost::function<void(std::shared_ptr<VisionNode>,
                                 std::shared_ptr<Data>)> callback_function);

  bool IsValid();
  void StartOneIteration(InputData input_data);
  virtual void Thread(InputData input_data);
  virtual void Init() {};

 protected:
  void call_tree_callback_function(std::shared_ptr<VisionNode> vision_node,
                                   std::shared_ptr<Data> data);
  virtual Data Function(InputData input_data) = 0;

 private:
  boost::thread *thread_;
  boost::function<void(std::shared_ptr<VisionNode>,
                       std::shared_ptr<Data>)> tree_callback_function_;

  // TODO(Keaven Martin) Add mutex on this data
  std::string id_;
  std::string tree_name_;
  Parameters parameters_;
  Dependences dependences_;
};

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_VISION_NODE_H_
