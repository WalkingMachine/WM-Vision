/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 02/04/2013
 *
 * Programmer: Keaven Martin
 *
 * Description: Node data thread-safe queue
 *
 */

#ifndef WM_VISION_INCLUDE_WM_VISIONKERNEL_THREAD_SAFE_QUEUE_H_
#define WM_VISION_INCLUDE_WM_VISIONKERNEL_THREAD_SAFE_QUEUE_H_

  #include <queue>
  #include "../include/wm_visionKernel/data.h"

  // TODO(Keaven Martin) Improve queue (dual queue)
  template<typename T>
  class ThreadSafeQueue {
    public:
      ~ThreadSafeQueue();

      void Enqueue(const T& node, Data data);
      T Dequeue(Data *data_output);

    private:
          std::queue<T> queue_;  // Use STL queue to store data
          std::queue<Data> queue_data_;
          boost::mutex mutex_;  // The mutex to synchronise on
          boost::condition_variable condition_;  // The condition to wait for
  };

  template<typename T>
  ThreadSafeQueue<T>::~ThreadSafeQueue() {
    condition_.notify_one();

    while (!queue_.empty()) {
      queue_.pop();
      queue_data_.pop();
    }
  }

  template<typename T>
  void ThreadSafeQueue<T>::Enqueue(const T& node, Data data) {
    // Acquire lock on the queue
    boost::unique_lock<boost::mutex> lock(mutex_);

    // Add the data to the queue
    queue_.push(node);
    queue_data_.push(data);

    // Notify others that data is ready
    condition_.notify_one();
  }

  template<typename T>
  T ThreadSafeQueue<T>::Dequeue(Data *data_output) {
    // Acquire lock on the queue
    boost::unique_lock<boost::mutex> lock(mutex_);

    // When there is no data, wait till someone fills it.
    // Lock is automatically released in the wait and obtained
    // again after the wait
    while (queue_.size() == 0 || queue_data_.size() == 0)
      condition_.wait(lock);

    // Retrieve the data from the queue
    T result = queue_.front();
    queue_.pop();
    data_output->set_data(queue_data_.front().data<void>());
    queue_data_.pop();
    return result;
  }

#endif  // WM_VISION_INCLUDE_WM_VISIONKERNEL_THREAD_SAFE_QUEUE_H_
