#include "thread_safe_utils/include/ThreadPool.h"
#include <stdexcept>

ThreadPool::ThreadPool(size_t threads)
    : threads(threads)
    , stop(false)
{
    for (size_t i = 0; i < threads; ++i)
        workers.emplace_back([this] {
            for (;;) {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(this->queue_mutex);
                    this->condition.wait(lock, [this] {
                        return this->stop || !this->taskQueue.empty();
                    });
                    if (this->stop && this->taskQueue.empty())
                        return;
                    task = std::move(this->taskQueue.front());
                    this->taskQueue.pop();
                }
                task();
            }
        });
}

ThreadPool::~ThreadPool()
{
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        stop = true;
    }
    condition.notify_all();
    for (std::thread& worker : workers) {
        if (worker.joinable())
            worker.join();
    }
}

inline size_t ThreadPool::getNumberOfThreads() const noexcept
{
    return this->threads;
}
