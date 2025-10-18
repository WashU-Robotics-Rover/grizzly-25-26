//
// Created by Daniel Huinda on 10/18/25.
//

#include <iostream>
#include <grizzly/core/runtime/scheduler.hpp>
#include <thread>

using namespace std::chrono_literals;

int main() {
    core::Scheduler scheduler;

    // Example: Register a task that prints "Hello, Grizzly!" every second
    scheduler.registerTask("hello_task", []() {
        std::cout << "Hello, Grizzly!" << std::endl;
    }, 1000ms, 1);

    int count = 0;
    scheduler.registerTask("count_task", [&count]() {
        std::cout << count << std::endl;
        count++;
    }, 100ms, 1);

    // Main loop
    while (true) {
        scheduler.tick();
        std::this_thread::sleep_for(1ms);
    }

    return 0;
}
