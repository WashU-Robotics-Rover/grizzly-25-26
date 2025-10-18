#ifndef SCHEDULER_HPP
#define SCHEDULER_HPP

#include <functional>
#include <vector>
#include <chrono>
#include <string>
#include <memory>
#include <mutex>

namespace core {

/**
 * @brief Represents a scheduled service task
 */
struct ScheduledTask {
    std::string name;
    std::function<void()> callback;
    std::chrono::milliseconds interval;
    std::chrono::steady_clock::time_point last_execution;
    bool enabled;
    int priority; // Higher number = higher priority

    ScheduledTask(const std::string& n,
                  std::function<void()> cb,
                  std::chrono::milliseconds interv,
                  int prio = 0)
        : name(n), callback(std::move(cb)), interval{interv},
          last_execution{std::chrono::steady_clock::now()},
          enabled{true}, priority{prio} {}
};

/**
 * @brief Scheduler class for managing periodic service execution
 *
 * This scheduler allows registration of services that need to run
 * at specific intervals. Useful for sensor polling, telemetry,
 * motor control updates, etc.
 */
class Scheduler {
public:
    Scheduler();
    ~Scheduler();

    /**
     * @brief Register a new task with the scheduler
     * @param name Unique identifier for the task
     * @param callback Function to execute
     * @param interval_ms Execution interval in milliseconds
     * @param priority Task priority (higher = more important)
     * @return true if task was registered successfully
     */
    bool registerTask(const std::string& name,
                     std::function<void()> callback,
                     std::chrono::milliseconds interval_ms,
                     int priority = 0);

    /**
     * @brief Unregister a task from the scheduler
     * @param name Name of the task to remove
     * @return true if task was found and removed
     */
    bool unregisterTask(const std::string& name);

    /**
     * @brief Enable a previously disabled task
     * @param name Name of the task to enable
     * @return true if task was found and enabled
     */
    bool enableTask(const std::string& name);

    /**
     * @brief Disable a task without removing it
     * @param name Name of the task to disable
     * @return true if task was found and disabled
     */
    bool disableTask(const std::string& name);

    /**
     * @brief Run one iteration of the scheduler
     *
     * Checks all registered tasks and executes those whose
     * interval has elapsed. Call this in your main loop.
     */
    void tick();

    /**
     * @brief Clear all registered tasks
     */
    void clearAllTasks();

    /**
     * @brief Get the number of registered tasks
     * @return Number of tasks (enabled and disabled)
     */
    size_t getTaskCount() const;

    /**
     * @brief Check if a task exists
     * @param name Name of the task
     * @return true if task is registered
     */
    bool hasTask(const std::string& name) const;

private:
    std::vector<std::shared_ptr<ScheduledTask>> tasks_;
    mutable std::mutex mutex_;

    void sortTasksByPriority();
};

} // namespace rover

#endif // SCHEDULER_HPP