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
void VisionNode::set_id(const std::string &id) {
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
void VisionNode::set_parameters(const Parameters &parameters) {
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
void VisionNode::set_dependences(const Dependences &dependences) {
  dependences_ = dependences;
}

std::string VisionNode::tree_name() {
  return tree_name_;
}

void VisionNode::set_tree_name(const std::string &tree_name) {
  tree_name_ = tree_name;
}

void VisionNode::set_tree_callback_function_(
    boost::function<void(std::shared_ptr<VisionNode>,
                         std::shared_ptr<Data>)> callback_function) {
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
void VisionNode::StartOneIteration(InputData input_data) {
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
void VisionNode::Thread(InputData input_data) {
  std::shared_ptr<Data> output_data(new Data(Function(input_data)));

  std::shared_ptr<VisionNode> this_vision_node(shared_from_this());
  // Call callback function
  tree_callback_function_(this_vision_node, output_data);
}

void VisionNode::call_tree_callback_function(
    std::shared_ptr<VisionNode> vision_node,
    std::shared_ptr<Data> data) {
  tree_callback_function_(vision_node, data);
}
