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
 * Description:  Tree's children
 *
 */

#include "../../include/vision_kernel/vision_node.h"

#include "../../include/vision_kernel/vision_node_factory.h"

VisionNodeFactory::MapType* VisionNodeFactory::map_;

/**
 * Default constructor
 */
VisionNode::VisionNode() {
  thread_ = nullptr;
}


/**
 * Destructor
 */
VisionNode::~VisionNode() {
  if (thread_ != nullptr)
        delete thread_;
}

/**
 * Get node identifier
 * @return
 */
std::string VisionNode::id() {
  return id_;
}

/**
 * Set node identifier
 * @param id
 */
void VisionNode::set_id(std::string id) {
  id_ = id;
}

/**
 * Get node parameters
 * @return
 */
VisionNode::Parameters VisionNode::parameters() {
  return parameters_;
}

/**
 * Set node parameters
 * @param parameters
 */
void VisionNode::set_parameters(Parameters parameters) {
  parameters_ = parameters;
}

/**
 * Get node dependences
 * @return
 */
VisionNode::Dependences VisionNode::dependences() {
  return dependences_;
}

/**
 * Set node dependences
 * @param dependences
 */
void VisionNode::set_dependences(Dependences dependences) {
  dependences_ = dependences;
}

std::string VisionNode::tree_name() {
  return tree_name_;
}

void VisionNode::set_tree_name(std::string tree_name) {
  tree_name_ = tree_name;
}

void VisionNode::set_tree_callback_function_(
    boost::function<void(VisionNode*, Data)> callback_function) {
  tree_callback_function_ = callback_function;
}

/**
 * Valid node conception (before usage)
 * @return validation
 */
bool VisionNode::IsValid() {
  // TODO(Keaven Martin) create correctly
  return true;
}

/**
 * Start the node function one time only
 * @param input_data
 */
void VisionNode::StartOneIteration(std::map<std::string, Data> *input_data) {
  if (thread_ != nullptr)
      thread_->join();

  delete thread_;

  thread_ = new boost::thread(boost::bind(&VisionNode::Thread,
                                          this,
                                          input_data));
}

/**
 * Node's thread
 * @param input_data
 */
void VisionNode::Thread(std::map<std::string, Data> *input_data) {
  Data outputData;
  outputData = Function(input_data);
  delete input_data;

  // Call callback function
  tree_callback_function_(this, outputData);
}

void VisionNode::call_tree_callback_function(VisionNode* vision_node, Data data) {
  tree_callback_function_(vision_node, data);
}
