/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 11/29/2012
 *
 * Programmer: Keaven Martin
 *
 * Description: Gestion of vision tree (creation and execution)
 *
 */

#ifndef WM_VISION_INCLUDE_WM_VISIONKERNEL_VISION_TREE_H_
#define WM_VISION_INCLUDE_WM_VISIONKERNEL_VISION_TREE_H_

  #include <vector>
  #include <string>
  #include <map>
  #include <ros/ros.h>

  #include "../include/wm_visionKernel/vision_node.h"
  #include "../include/wm_visionKernel/thread_safe_queue.h"
  #include "../include/wm_visionKernel/data.h"

  class VisionTree {
    // TODO(Keaven Martin) Add mutex lock when we add some nodes
    public:
      typedef std::map<std::string, std::string> Dependences;
      typedef std::map<std::string, std::string> Parameters;

      explicit VisionTree(std::string name, float wanted_frequency);
      ~VisionTree();

      bool AddNode(std::string type,
                   std::string id,
                   Dependences dependences,
                   Parameters parameters,
                   std::string debug_node = "");
      bool IsValid();
      void CallbackFunction(VisionNode* visionNode, Data outputData);
      void Start();
      void Stop();
      void Thread();

      void set_wanted_frequency(float wanted_frequency);
      float wanted_frequency();
      float current_frequency();
      void set_name(std::string name);
      std::string name();

    private:
      std::string name_;
      std::vector<VisionNode*> vision_nodes_;
      ThreadSafeQueue<VisionNode*> *queue_;
      boost::thread *thread_;
      bool must_stop_;
      float wanted_frequency_;
      float current_frequency_;
      boost::mutex must_stop_mutex_;
      boost::mutex wanted_frequency_mutex_;
      boost::mutex current_frequency_mutex_;

      void Process();
  };

#endif  // WM_VISION_INCLUDE_WM_VISIONKERNEL_VISION_TREE_H_
