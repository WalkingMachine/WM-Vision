/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision_kernel
 *
 * Creation date: 12/06/2012
 *
 * Programmer: Keaven Martin
 *
 * Description: Flow's children. A vision node represent a threaded function.
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
 * @return identifier
 */
std::string VisionNode::id() {
  return id_;
}

/**
 * Set node identifier
 * @param identifier
 */
void VisionNode::set_id(const std::string &id) {
  id_ = id;
}

/**
 * Get node parameters
 * @return parameters
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
 * @return dependences
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

/**
 * Get vision flow name
 * @return Vision flow name
 */
std::string VisionNode::flow_name() {
  return flow_name_;
}

/**
 * Set vision flow name
 * @param flow_name
 */
void VisionNode::set_flow_name(const std::string &flow_name) {
  flow_name_ = flow_name;
}

/**
 * Set vision flow callback (called at the end of the node function)
 * @param callback_function
 */
void VisionNode::set_flow_callback_function_(
    boost::function<void(std::shared_ptr<VisionNode>,
                         std::shared_ptr<Data>)> callback_function) {
  flow_callback_function_ = callback_function;
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
  thread_mutex_.lock();
  input_data_ = input_data;
  thread_mutex_.unlock();

  must_stop_ = false;

  if (thread_ == nullptr) {  // First time
    thread_ = new boost::thread(boost::bind(&VisionNode::Thread, this));
  } else {
    thread_start_condition_.notify_one();
  }
}

/**
 * Stop vision node (at the end of the node function)
 */
void VisionNode::Stop() {
      must_stop_ = true;
      thread_start_condition_.notify_one();

      boost::mutex::scoped_lock lock(thread_mutex_);
      thread_stoped_condition_.timed_wait(lock, boost::posix_time::milliseconds(1));
}

/**
 * Node's thread
 */
void VisionNode::Thread() {
  do {
    boost::mutex::scoped_lock lock(thread_mutex_);

    std::shared_ptr<Data> output_data(new Data(Function(input_data_))); // Start node function and output data

    std::shared_ptr<VisionNode> this_vision_node(shared_from_this());
    flow_callback_function_(this_vision_node, output_data);  // Call callback function

    thread_start_condition_.wait(lock);
  } while(!must_stop_);
  thread_stoped_condition_.notify_one();
}

/**
 * Force call of vision flow callback
 * @param vision_node Sender (Vision node)
 * @param data Vision node data
 */
void VisionNode::call_flow_callback_function(
    std::shared_ptr<VisionNode> vision_node,
    std::shared_ptr<Data> data) {
  flow_callback_function_(vision_node, data);
}
