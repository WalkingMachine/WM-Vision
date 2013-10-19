/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision_kernel
 *
 * Creation date: 02/26/2013
 *
 * Programmer: Julien Côté
 *
 * Description: ROS Topic input manager
 *
 */

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_INPUT_MANAGER_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_INPUT_MANAGER_H_

#include <memory>
#include <mutex>
#include <map>
#include <string>
#include <ros/subscriber.h>

#include "input.h"

/**
 * ROS Topic input manager singleton (where the template is the topic type)
 * One InputManager by topic message type
 */
template <class T>
class InputManager {
 public:
  /**
   * Get singleton instance
   * @return
   */
  static InputManager<T>& GetInstance() {
    static InputManager<T> instance_;
    return instance_;
  };

  /**
   *
   * @param topic_name
   * @return
   */
  std::shared_ptr<T> GetInput(const std::string &topic_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto input_iterator = input_list_.find(topic_name);
    return input_iterator->second->Poll();
  };

  /**
   * Subscribe to a new ROS Topic
   * @param topic_name
   */
  void AddSubscriber(const std::string &topic_name) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto input_iterator = input_list_.find(topic_name);
    if (input_iterator == input_list_.end()) {
      std::shared_ptr<Input<T>> input =
          std::shared_ptr<Input<T>>(new Input<T>(topic_name));
      input_list_[topic_name] = input;
    } else {
      input_iterator->second->AddSubscriber();
    }
  };

  /**
   * Verify if a ROS Topic has new data
   * @param topic_name
   * @return True if the topic has some new data
   */
  bool has_data(const std::string &topic_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto input_iterator = input_list_.find(topic_name);
    return input_iterator->second->has_data();
  };

  /**
   * Remove subscription to a ROS Topic
   * @param topic_name
   */
  void RemoveSubscriber(const std::string &topic_name) {
   std::lock_guard<std::mutex> lock(mutex_);

    auto input_iterator = input_list_.find(topic_name);
    if (input_iterator == input_list_.end()) {
      std::exception e;
      throw e;
    } else {
      int remaining_subscribers;
      remaining_subscribers = input_iterator->second->RemoveSubscriber();
      if (remaining_subscribers == 0) {
        input_list_.erase(input_iterator);
      }
    }
  };

 private:
  std::mutex mutex_;
  std::map<std::string, std::shared_ptr<Input<T>>> input_list_;

  // Required private members for the singleton
  /**
   * Constructor
   */
  InputManager() {};

  /**
   * Copy Constructor
   * @param
   */
  InputManager(const InputManager<T>&) {};

  /**
   * Copy operator
   * @param
   * @return
   */
  InputManager& operator= (const InputManager<T>&);
};

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_INPUT_MANAGER_H_
