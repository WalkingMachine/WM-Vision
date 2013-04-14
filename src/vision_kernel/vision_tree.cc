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
 * Description: Gestion of vision tree (creation and execution)
 *
 */

#include "../../include/vision_kernel/vision_tree.h"

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
VisionTree::VisionTree(const std::string &name, const float &wanted_frequency) {
  must_stop_ = true;
  thread_ = nullptr;
  queue_ = nullptr;

  set_name(name);
  set_wanted_frequency(wanted_frequency);
  current_frequency_ = 0;
}

/**
 * Destructor
 */
VisionTree::~VisionTree() {
  Stop();

  // Delete content in visonNodes vector
  for (std::vector<VisionNode*>::iterator it = vision_nodes_.begin();
      it != vision_nodes_.end(); ) {
    delete *it;
    it = vision_nodes_.erase(it);
  }
}

/**
 * Add one node at the tree before start
 * @param type VisionNode type(name)
 * @param id
 * @param dependences
 * @param parameters
 * @return Validation
 */
bool VisionTree::AddNode(const std::string &type,
                         const std::string &id,
                         const Dependences &dependences,
                         const Parameters &parameters,
                         const std::string &debug_node) {
  VisionNode *vision_node = VisionNodeFactory::CreateInstance(type);

  vision_node->set_id(id);
  vision_node->set_tree_name(name());
  vision_node->set_parameters(parameters);
  vision_node->set_dependences(dependences);

  if (debug_node.empty()) {
    vision_node->set_tree_callback_function_(
          boost::bind(&VisionTree::CallbackFunction, this, _1, _2));
    vision_nodes_.push_back(vision_node);
  } else {
    VisionDebugNode *vision_debug_node = (VisionDebugNode *)VisionNodeFactory::CreateInstance(debug_node);

    vision_debug_node->set_id(id);
    vision_debug_node->set_tree_name(name());
    vision_debug_node->set_parameters(parameters);
    vision_debug_node->set_dependences(dependences);
    vision_debug_node->set_vision_node(vision_node);
    vision_debug_node->set_tree_callback_function_(
              boost::bind(&VisionTree::CallbackFunction, this, _1, _2));
    vision_node->set_tree_callback_function_(
              boost::bind(&VisionDebugNode::CallbackFunction, vision_debug_node, _1, _2));
    vision_debug_node->Init();
    vision_nodes_.push_back(vision_debug_node);
  }
  vision_node->Init();

  // TODO(Keaven Martin) Validate

  return true;
}

// TODO(Keaven Martin) Make
/**
 * Valid vision tree
 * @return
 */
bool VisionTree::IsValid() {
  return true;
}

/**
 * Callback function use when a node is finished
 * @param visionNode
 * @param outputData
 */
void VisionTree::CallbackFunction(VisionNode* visionNode, const Data &node_output_data) {
  // Add visionNode and data in Queue
  queue_->Enqueue(visionNode, node_output_data);
}

/**
 * Start vision tree
 */
void VisionTree::Start() {
  // TODO(Keaven Martin) Valid thread is not running
  must_stop_ = false;
  queue_ = new ThreadSafeQueue<VisionNode*>;
  thread_ = new std::thread(boost::bind(&VisionTree::Thread, this));
}

/**
 * Stop vision tree
 */
void VisionTree::Stop() {
  if (!must_stop_) {
    // Signal the thread to stop (thread-safe)
    must_stop_mutex_.lock();
      must_stop_ = true;
    must_stop_mutex_.unlock();

    // Wait for the thread to finish.
    if (thread_ != nullptr)
      thread_->join();

    delete thread_;
    delete queue_;
  }
}

/**
 * Set wanted vision tree process frequency
 * @param wanted_frequency
 */
void VisionTree::set_wanted_frequency(const float &wanted_frequency) {
  std::lock_guard<std::mutex> lock(wanted_frequency_mutex_);
  wanted_frequency_ = wanted_frequency;
}

/**
 * Get wanted vision tree process frequency
 * @return wanted frequency
 */
float VisionTree::wanted_frequency() {
  std::lock_guard<std::mutex> lock(wanted_frequency_mutex_);
  return wanted_frequency_;
}

/**
 * Get current vision tree process frequency
 * @return current frequency
 */
float VisionTree::current_frequency() {
  std::lock_guard<std::mutex> lock(current_frequency_mutex_);
  return current_frequency_;
}

void VisionTree::set_name(std::string name) {
  name_ = name;
}

std::string VisionTree::name() {
  return name_;
}

/**
 * Thread base function
 */
void VisionTree::Thread() {
  bool must_stop;

  wanted_frequency_mutex_.lock();
  ros::Rate frequency(wanted_frequency_);
  wanted_frequency_mutex_.unlock();

  ros::Time currentTime;
  ros::Time lastTime = ros::Time::now();

  do {
    Process();

    // Sleep some time base on frequency
    frequency.sleep();

    currentTime = ros::Time::now();

    // Set current frequency (thread safe)
    current_frequency_mutex_.lock();
      current_frequency_ = 1000000000.0 / (currentTime - lastTime).toNSec();
    current_frequency_mutex_.unlock();

    lastTime = currentTime;

    // Get the "must stop" state (thread safe)
    must_stop_mutex_.lock();
      must_stop = must_stop_;
    must_stop_mutex_.unlock();
  } while (must_stop == false);
}

/**
 * Thread process for vision nodes gestion
 */
// TODO(Keaven Martin) Comment
void VisionTree::Process() {
  // First time take no dependence nodes
  for (VisionNode *it : vision_nodes_) {
    if (it->dependences().empty()) {
      it->StartOneIteration(nullptr);
    }
  }

  VisionNode* visionNodeFinished;
  std::map<VisionNode*, Data> visionNodesFinishedWithData;

  bool continu = true;

  while (continu) {
    Data olderData;
    visionNodeFinished = queue_->Dequeue(&olderData);
    visionNodesFinishedWithData.insert(
        std::pair<VisionNode*, Data>(visionNodeFinished, olderData));

    // If all nodes was finished
    if (visionNodesFinishedWithData.size() == vision_nodes_.size()) {
      continu = false;  // Quit
    } else {
      for (VisionNode *vision_node_iteration : vision_nodes_) {
        uint numberNodeWithThisDependence = 0;
        uint numberNodeWithFinishedDependence = 0;
        bool visionNodeAlreadyFinished = false;

        for (std::pair<VisionNode*, Data> vision_node_finished_iteration : visionNodesFinishedWithData) {
          if (vision_node_iteration->id() == vision_node_finished_iteration.first->id()) {
            visionNodeAlreadyFinished = true;
          }
        }

        if(!visionNodeAlreadyFinished) {
          for (std::pair<std::string, std::string> vision_node_dependence_iteration : vision_node_iteration->dependences()) {
            if (visionNodeFinished->id() == vision_node_dependence_iteration.second.data()) {
              numberNodeWithThisDependence++;
            } else {
              for (std::pair<VisionNode*, Data> vision_node_finished_iteration : visionNodesFinishedWithData) {
                if (vision_node_dependence_iteration.second.data() == vision_node_finished_iteration.first->id()) {
                  numberNodeWithFinishedDependence++;
                }
              }
            }

            if (vision_node_iteration->dependences().size() == numberNodeWithThisDependence + numberNodeWithFinishedDependence) {
              std::map<std::string, Data> *inputData = new std::map<std::string, Data>;

              for(std::pair<std::string, std::string> value : vision_node_iteration->dependences()) {
                for(std::pair<VisionNode*, Data> vision_node_finished_iteration : visionNodesFinishedWithData) {
                  if (value.second.data() == vision_node_finished_iteration.first->id()) {
                    inputData->insert(std::pair<std::string, Data>(
                                          value.first.data(),
                                          vision_node_finished_iteration.second));
                  }
                }
              }

              vision_node_iteration->StartOneIteration(inputData);
            }
          }
        }
      }
    }
  }

  visionNodesFinishedWithData.clear();
}
