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
 * Description: ROS Topic input
 *
 */

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_INPUT_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_INPUT_H_

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <mutex>
#include <boost/shared_ptr.hpp>

/**
 * ROS Topic input
 * One for each topic
 */
template <class T>
class Input {
 public:
  /**
   * Constructor
   * @param topic_name
   */
  Input(const std::string &topic_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    ros::NodeHandle node_handle;
    bool debug_mode;
    has_data_ = false;

    if (!node_handle.getParamCached("debug_mode", debug_mode)) {
      debug_mode = true;
    }

    if (debug_mode) {
      DebugTopicNameValidation(topic_name);
    }

   boost::function<void(boost::shared_ptr<T>)> callback_function =
       boost::bind(&Input::CallBackSubscriber, this, _1);

   subsciber_ = node_handle.subscribe<boost::shared_ptr<T>>(topic_name,
                                         10,
                                         callback_function);

    subscriber_count_ = 1;
  }

  /**
   * Give the last message received
   * @return Last message in stack
   */
  std::shared_ptr<T> Poll() {
//    std::lock_guard<std::mutex> lock(mutex_);
    return last_message_;
  }

  /**
   * Call back function use when we received a new message
   * @param message ROS Topic message (the template is the message type)
   */
  void CallBackSubscriber(const boost::shared_ptr<T> message) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_message_ = to_std(message);
    has_data_ = true;
  }

  /**
   * Add a subscriber to this ROS Topic
   */
  void AddSubscriber() {
    std::lock_guard<std::mutex> lock(mutex_);
    subscriber_count_++;
  };

  /**
   * Remove a subscriber to this ROS Topic
   * @return Subscriber count after removing one subscriber
   */
  int RemoveSubscriber() {
    std::lock_guard<std::mutex> lock(mutex_);
    subscriber_count_--;
    return subscriber_count_; // TODO(Keaven Martin): remove and add SubscriberCount function
  };

  /**
   * Verify if the ROS Topic has new data
   * @return True if the topic has some new data
   */
  bool has_data() {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_data_;
  };

 private:
  std::mutex mutex_;
  int subscriber_count_;
  ros::Subscriber subsciber_;
  std::shared_ptr<T> last_message_;
  bool has_data_;

  /**
   * Convert boost::shared_ptr to std::shared_ptr
   * @param p boost pointer
   * @return
   */
  std::shared_ptr<T> to_std(const boost::shared_ptr<T> &p) {
      return std::shared_ptr<T>(p.get(), [p](...){});
  }

  /**
   * Valid the existence of a topic (in debug mode only)
   * @param topic_name
   */
  void DebugTopicNameValidation(const std::string &topic_name) {
    std::vector<ros::master::TopicInfo> topics;
    ros::master::getTopics(topics);

    bool topic_exists;
    for (ros::master::TopicInfo &topic : topics) {
      if (topic.name == topic_name) {
        topic_exists = true;
      }
    }

    if (!topic_exists) {
      ROS_ERROR("Input: with name %s doesn't exist", topic_name.c_str());
    }
  }
};

#endif // WM_VISION_INCLUDE_VISION_KERNEL_INPUT_H_
