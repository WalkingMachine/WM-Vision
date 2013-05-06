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

#include "../../include/vision_kernel/vision_flow.h"

#include <functional>
#include <boost/bind.hpp>

#include "../../include/vision_kernel/vision_node_factory.h"
#include "../../include/vision_kernel/vision_debug_node.h"

#include "../../include/vision_kernel/vision_nodes/grayscale_node.h"
#include "../../include/vision_kernel/vision_nodes/threshold_node.h"
#include "../../include/vision_kernel/vision_nodes/openni_node.h"
#include "../../include/vision_kernel/vision_nodes/edge_detection_node.h"
#include "../../include/vision_kernel/vision_nodes/line_detection_node.h"


/**
 * Constructor
 * @param wanted_frequency
 */
VisionFlow::VisionFlow(const std::string &name, const float &wanted_frequency) {
  must_stop_ = true;
  thread_ = nullptr;

  set_name(name);
  set_wanted_frequency(wanted_frequency);
  current_frequency_ = 0;
}

/**
 * Destructor
 */
VisionFlow::~VisionFlow() {
  Stop();

  // Delete content in visonNodes vector
  vision_nodes_.clear();
}

/**
 * Add one node at the flow before start
 * @param type VisionNode type(name)
 * @param id
 * @param dependences
 * @param parameters
 * @return Validation
 */
bool VisionFlow::AddNode(const std::string &type,
                         const std::string &id,
                         const Dependences &dependences,
                         const Parameters &parameters,
                         const std::string &debug_node) {
  std::shared_ptr<VisionNode> vision_node =
      VisionNodeFactory::CreateInstance(type);

  vision_node->set_id(id);
  vision_node->set_flow_name(name());
  vision_node->set_parameters(parameters);
  vision_node->set_dependences(dependences);

  if (debug_node.empty()) {
    vision_node->set_flow_callback_function_(
          boost::bind(&VisionFlow::CallbackFunction, this, _1, _2));
    vision_nodes_.push_back(vision_node);
  } else {
    std::shared_ptr<VisionDebugNode> vision_debug_node(std::static_pointer_cast<VisionDebugNode>(VisionNodeFactory::CreateInstance(debug_node)));

    vision_debug_node->set_id(id);
    vision_debug_node->set_flow_name(name());
    vision_debug_node->set_parameters(parameters);
    vision_debug_node->set_dependences(dependences);
    vision_debug_node->set_vision_node(vision_node);
    vision_debug_node->set_flow_callback_function_(
              boost::bind(&VisionFlow::CallbackFunction, this, _1, _2));
    vision_node->set_flow_callback_function_(
              boost::bind(&VisionDebugNode::CallbackFunction, vision_debug_node.get(), _1, _2));
    vision_debug_node->Init();
    vision_nodes_.push_back(vision_debug_node);
  }
  vision_node->Init();

  // TODO(Keaven Martin) Validate

  return true;
}

// TODO(Keaven Martin) Make
/**
 * Valid vision flow
 * @return
 */
bool VisionFlow::IsValid() {
  return true;
}

/**
 * Callback function use when a node is finished
 * @param visionNode
 * @param outputData
 */
void VisionFlow::CallbackFunction(std::shared_ptr<VisionNode> vision_node,
                                  std::shared_ptr<Data> node_output_data) {
  // Add visionNode and data in Queue
  NodeThreadSafeQueue::NodeWithData node(vision_node, node_output_data);
  queue_.Enqueue(node);
}

/**
 * Start vision flow
 */
void VisionFlow::Start() {
  // TODO(Keaven Martin) Valid thread is not running
  must_stop_ = false;
  thread_ = new std::thread(boost::bind(&VisionFlow::Thread, this));
}

/**
 * Stop vision flow
 */
void VisionFlow::Stop() {
  if (!must_stop_) {
    // Signal the thread to stop (thread-safe)
    must_stop_mutex_.lock();
      must_stop_ = true;
    must_stop_mutex_.unlock();

    // Wait for the thread to finish.
    if (thread_ != nullptr)
      thread_->join();

    delete thread_;

    queue_.Clear();
  }
}

/**
 * Set wanted vision flow process frequency
 * @param wanted_frequency
 */
void VisionFlow::set_wanted_frequency(const float &wanted_frequency) {
  std::lock_guard<std::mutex> lock(wanted_frequency_mutex_);
  wanted_frequency_ = wanted_frequency;
}

/**
 * Get wanted vision flow process frequency
 * @return wanted frequency
 */
float VisionFlow::wanted_frequency() {
  std::lock_guard<std::mutex> lock(wanted_frequency_mutex_);
  return wanted_frequency_;
}

/**
 * Get current vision flow process frequency
 * @return current frequency
 */
float VisionFlow::current_frequency() {
  std::lock_guard<std::mutex> lock(current_frequency_mutex_);
  return current_frequency_;
}

void VisionFlow::set_name(std::string name) {
  name_ = name;
}

std::string VisionFlow::name() {
  return name_;
}

/**
 * Thread base function
 */
void VisionFlow::Thread() {
  bool must_stop;

  wanted_frequency_mutex_.lock();
  ros::Rate frequency(wanted_frequency_);
  wanted_frequency_mutex_.unlock();

  ros::Time current_time;
  ros::Time last_time = ros::Time::now();

  do {
    Process();

    // Sleep some time base on frequency
    frequency.sleep();

    current_time = ros::Time::now();

    // Set current frequency (thread safe)
    current_frequency_mutex_.lock();
      current_frequency_ = 1000000000.0 / (current_time - last_time).toNSec();
    current_frequency_mutex_.unlock();

    last_time = current_time;

    // Get the "must stop" state (thread safe)
    must_stop_mutex_.lock();
      must_stop = must_stop_;
    must_stop_mutex_.unlock();
  } while (must_stop == false);
}

/**
 * Thread process for vision nodes gestion
 */
// TODO(Keaven Martin) Comment and clean
void VisionFlow::Process() {
  // First time take no dependence nodes
  for (auto &vision_node : vision_nodes_) {
    if (vision_node->dependences().empty()) {
      vision_node->StartOneIteration(nullptr);
    }
  }

  NodeThreadSafeQueue::NodeWithData older_node;
  std::map<std::shared_ptr<VisionNode>, std::shared_ptr<Data>> vision_nodes_finished_with_data;

  bool continu = true;

  while (continu) {
    older_node = queue_.Dequeue();
    vision_nodes_finished_with_data.insert(older_node);

    // If all nodes was finished
    if (vision_nodes_finished_with_data.size() == vision_nodes_.size()) {
      continu = false;  // Quit
    } else {
      for (auto &vision_node_iteration : vision_nodes_) {
        uint number_node_with_this_dependence = 0;
        uint number_node_with_finished_dependence = 0;
        bool vision_node_already_finished = false;

        for (auto &vision_node_finished_iteration : vision_nodes_finished_with_data) {
          if (vision_node_iteration->id() == vision_node_finished_iteration.first->id()) {
            vision_node_already_finished = true;
          }
        }

        if(!vision_node_already_finished) {
          for (auto &vision_node_dependence_iteration : vision_node_iteration->dependences()) {
            if (older_node.first->id() == vision_node_dependence_iteration.second.data()) {
              number_node_with_this_dependence++;
            } else {
              for (auto &vision_node_finished_iteration : vision_nodes_finished_with_data) {
                if (vision_node_dependence_iteration.second.data() == vision_node_finished_iteration.first->id()) {
                  number_node_with_finished_dependence++;
                }
              }
            }

            if (vision_node_iteration->dependences().size() == number_node_with_this_dependence + number_node_with_finished_dependence) {
              VisionNode::InputData input_data(new std::map<std::string, std::shared_ptr<Data>>);

              for(auto &value : vision_node_iteration->dependences()) {
                for(auto &vision_node_finished_iteration : vision_nodes_finished_with_data) {
                  if (value.second.data() == vision_node_finished_iteration.first->id()) {
                    (*input_data)[value.first.data()] = vision_node_finished_iteration.second;
                  }
                }
              }
              vision_node_iteration->StartOneIteration(input_data);
            }
          }
        }
      }
    }
  }

  vision_nodes_finished_with_data.clear();
}
