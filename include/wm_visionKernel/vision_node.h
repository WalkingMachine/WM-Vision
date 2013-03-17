/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 12/06/2012
 *
 * Programmer: Keaven Martin
 *
 * Description: Tree's children
 *
 */

#ifndef WM_VISION_INCLUDE_WM_VISIONKERNEL_VISION_NODE_H_
#define WM_VISION_INCLUDE_WM_VISIONKERNEL_VISION_NODE_H_

  #include <boost/thread.hpp>
  #include <string>
  #include <map>

  #include "../include/wm_visionKernel/data.h"

  class VisionNode {
    public:
      // Type definition
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
      void set_id(std::string id);
      void set_tree_name(std::string tree_name);
      void set_parameters(Parameters parameters);
      void set_dependences(Dependences dependences);
      void set_tree_callback_function_(
                boost::function<void(VisionNode*, Data)> callback_function);

      bool IsValid();
      void StartOneIteration(std::map<std::string, Data> *input_data);
      virtual void Thread(std::map<std::string, Data> *input_data);
      virtual void Init() {};

    protected:
      void call_tree_callback_function(VisionNode* vision_node, Data data);
      virtual Data Function(std::map<std::string, Data> *input_data) = 0;

    private:
      boost::thread* thread_;
      boost::function<void(VisionNode*, Data)> tree_callback_function_;

      // TODO(Keaven Martin) Add mutex on this data
      std::string id_;
      std::string tree_name_;
      Parameters parameters_;
      Dependences dependences_;
  };

#endif  // WM_VISION_INCLUDE_WM_VISIONKERNEL_VISION_NODE_H_