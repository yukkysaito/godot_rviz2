//
//  Copyright 2022 Yukihiro Saito. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

/**
 * @class AsyncTask
 * @brief Runs one heavy job at a time on a worker thread, so that the render loop never blocks.
 *
 * The owner polls is_done() from the main thread and then take()s the result. The destructor
 * waits for a running job, so declare the task after the members its job uses.
 */
template <class ResultT>
class AsyncTask
{
public:
  AsyncTask() = default;
  AsyncTask(const AsyncTask &) = delete;
  AsyncTask & operator=(const AsyncTask &) = delete;
  ~AsyncTask()
  {
    if (thread_.joinable()) thread_.join();
  }

  /**
   * @brief Starts job on the worker thread. Returns false (and does nothing) while a job runs.
   */
  bool start(std::function<ResultT()> job)
  {
    if (running_) return false;
    if (thread_.joinable()) thread_.join();
    running_ = true;
    done_ = false;
    thread_ = std::thread([this, job = std::move(job)]() {
      ResultT result = job();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        result_ = std::move(result);
      }
      done_ = true;
      running_ = false;
    });
    return true;
  }

  bool is_running() const { return running_; }

  /**
   * @brief True when a job finished and its result has not been taken yet.
   */
  bool is_done() const { return done_; }

  ResultT take()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    done_ = false;
    return std::exchange(result_, ResultT());
  }

private:
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> done_{false};
  std::mutex mutex_;
  ResultT result_;
};
