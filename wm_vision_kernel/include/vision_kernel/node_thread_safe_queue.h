/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: vision_kernel
 *
 * Creation date: 02/04/2013
 *
 * Programmer: Keaven Martin
 *
 * Description: Node data thread-safe queue
 *
 */

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_NODE_THREAD_SAFE_QUEUE_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_NODE_THREAD_SAFE_QUEUE_H_

#include <queue>
#include <memory>
#include <mutex>
#include <condition_variable>

#include "vision_node.h"
#include "data.h"

class NodeThreadSafeQueue {
 public:
  typedef std::pair<std::shared_ptr<VisionNode>, std::shared_ptr<Data>> NodeWithData;
  ~NodeThreadSafeQueue() {
    Clear();
  }

  void Enqueue(NodeWithData node_with_data) {
    // Acquire lock on the queue
    std::unique_lock<std::mutex> lock(mutex_);

    // Add the data to the queue
    queue_.push(node_with_data);

    // Notify others that data is ready
    condition_.notify_one();
  }

  NodeWithData Dequeue() {
    // Acquire lock on the queue
    std::unique_lock<std::mutex> lock(mutex_);

    // When there is no data, wait till someone fills it.
    // Lock is automatically released in the wait and obtained
    // again after the wait
    while (queue_.size() == 0)
      condition_.wait(lock);

    // Retrieve the data from the queue
    NodeWithData result = queue_.front();
    queue_.pop();

    return result;
  }

  void Clear() {
    std::unique_lock<std::mutex> lock(mutex_);

    while (!queue_.empty()) {
      queue_.pop();
    }
  }

 private:
  std::queue<NodeWithData> queue_;  // Use STL queue to store data
  std::mutex mutex_;  // The mutex to synchronise on
  std::condition_variable condition_;  // The condition to wait for
};

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_NODE_THREAD_SAFE_QUEUE_H_
