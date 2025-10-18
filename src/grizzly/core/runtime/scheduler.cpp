#include "grizzly/core/runtime/scheduler.hpp"
#include <algorithm>

namespace core {

Scheduler::Scheduler() {}

Scheduler::~Scheduler() {
    clearAllTasks();
}

bool Scheduler::registerTask(const std::string& name,
                            std::function<void()> callback,
                            std::chrono::milliseconds interval_ms,
                            int priority) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Check if task already exists
    for (const auto& task : tasks_) {
        if (task->name == name) {
            return false;
        }
    }

    auto task = std::make_shared<ScheduledTask>(
        name,
        callback,
        interval_ms,
        priority
    );

    tasks_.push_back(task);
    sortTasksByPriority();

    return true;
}

bool Scheduler::unregisterTask(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = std::remove_if(tasks_.begin(), tasks_.end(),
        [&name](const std::shared_ptr<ScheduledTask>& task) {
            return task->name == name;
        });

    if (it != tasks_.end()) {
        tasks_.erase(it, tasks_.end());
        return true;
    }

    return false;
}

bool Scheduler::enableTask(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto& task : tasks_) {
        if (task->name == name) {
            task->enabled = true;
            task->last_execution = std::chrono::steady_clock::now();
            return true;
        }
    }

    return false;
}

bool Scheduler::disableTask(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto& task : tasks_) {
        if (task->name == name) {
            task->enabled = false;
            return true;
        }
    }

    return false;
}

void Scheduler::tick() {
    std::lock_guard<std::mutex> lock(mutex_);

    auto now = std::chrono::steady_clock::now();

    for (auto& task : tasks_) {
        if (!task->enabled) {
            continue;
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - task->last_execution
        );

        if (elapsed >= task->interval) {
            try {
                task->callback();
                task->last_execution = now;
            } catch (const std::exception& e) {
                // Log error but continue execution
                // Consider adding proper logging here
            }
        }
    }
}

void Scheduler::clearAllTasks() {
    std::lock_guard<std::mutex> lock(mutex_);
    tasks_.clear();
}

size_t Scheduler::getTaskCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return tasks_.size();
}

bool Scheduler::hasTask(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);

    for (const auto& task : tasks_) {
        if (task->name == name) {
            return true;
        }
    }

    return false;
}

void Scheduler::sortTasksByPriority() {
    std::sort(tasks_.begin(), tasks_.end(),
        [](const std::shared_ptr<ScheduledTask>& a,
           const std::shared_ptr<ScheduledTask>& b) {
            return a->priority > b->priority;
        });
}

} // namespace rover