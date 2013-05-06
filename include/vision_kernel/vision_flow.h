/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: vision_kernel
 *
 * Creation date: 11/29/2012
 *
 * Programmer: Keaven Martin
 *
 * Description: Gestion of vision flow (creation and execution)
 *
 */

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_VISION_FLOW_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_VISION_FLOW_H_

#include <vector>
#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <ros/ros.h>

#include "vision_node.h"
#include "node_thread_safe_queue.h"
#include "data.h"

// TODO(Keaven Martin) Add mutex lock when we add some nodes
class VisionFlow {
 public:
  typedef std::map<std::string, std::string> Dependences;
  typedef std::map<std::string, std::string> Parameters;

  VisionFlow(const std::string &name, const float &wanted_frequency);
  ~VisionFlow();

  bool AddNode(const std::string &type,
               const std::string &id,
               const Dependences &dependences,
               const Parameters &parameters,
               const std::string &debug_node = "");
  bool IsValid();
  void CallbackFunction(std::shared_ptr<VisionNode> vision_node,
                        std::shared_ptr<Data> node_output_data);
  void Start();
  void Stop();
  void Thread();

  void set_wanted_frequency(const float &wanted_frequency);
  float wanted_frequency();
  float current_frequency();
  void set_name(std::string name);
  std::string name();

 private:
  std::string name_;
  std::vector<std::shared_ptr<VisionNode>> vision_nodes_;
  NodeThreadSafeQueue queue_;
  std::thread *thread_;
  bool must_stop_;
  float wanted_frequency_;
  float current_frequency_;
  std::mutex must_stop_mutex_;
  std::mutex wanted_frequency_mutex_;
  std::mutex current_frequency_mutex_;
  int shared_ptr;

  void Process();
};

#endif  // WM_VISION_INCLUDE_WM_VISIONKERNEL_VISION_FLOW_H_
