/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 02/26/2013
 *
 * Programmer: Julien Côté
 *
 * Description:
 *
 */

#ifndef INPUT_H_
#define INPUT_H_

#include <ros/ros.h>
#include <boost/bind.hpp>

template <class T>
class Input {
 public:
  Input(std::string topic_name){
    ros::NodeHandle node_handle;
    bool debug_mode;
    has_data_ = false;

    node_handle.getParamCached("debug_mode", debug_mode);

    if(debug_mode){
      DebugTopicNameValidation(topic_name);
    }
   boost::function<void(T)> callback_function = boost::bind(&Input::CallBackSubscriber, this, _1);

   subsciber_ = node_handle.subscribe<T>(topic_name,
                                         1000,
                                         callback_function);

    subscriber_count_ = 1;
  }

  T Poll(){
    return last_message_;
  }

  void CallBackSubscriber(T message){
    last_message_ = message;
    has_data_ = true;
  }

  void AddSubscriber(){
    ROS_INFO("AddSubscriber");
    subscriber_count_++;
  };

  int RemoveSubscriber(){
    ROS_INFO("RemoveSubscriber");
    subscriber_count_--;
    return subscriber_count_;
  };

  bool has_data(){
    return has_data_;
  };

 private:
  int subscriber_count_;
  ros::Subscriber subsciber_;
  T last_message_;
  bool has_data_;

  void DebugTopicNameValidation(std::string topic_name) {
    std::vector<ros::master::TopicInfo> topics;
    ros::master::getTopics(topics);

    bool topic_exists;
    for(ros::master::TopicInfo &topic : topics) {
      if(topic.name == topic_name) {
        topic_exists = true;
      }
    }

    if(!topic_exists) {
      std::exception e;
      throw e;
      // TODO(Julien Cote) Do something intelligent with this error
    }
  }
};

#endif /* INPUT_H_ */
