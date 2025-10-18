//
// Created by Daniel Huinda on 10/18/25.
//

#pragma once

#include <chrono>
#include <string>
#include <vector>

using namespace std;

using time_point = chrono::steady_clock::time_point;
using nanoseconds = chrono::nanoseconds;

// Structure representing a loop section
struct IClock {
    virtual ~IClock() = default;
    virtual time_point now() const = 0;
};

// Steady clock implementation
struct SteadyClock final : public IClock {
    time_point now() const override { return std::chrono::steady_clock::now(); }
};


struct LoopSpec {
    string name;
    int hz = 50;
    int priority = 0;
    struct ILoop * loop = nullptr;
};

// Interface for loops
struct Iloop {
    virtual ~Iloop() = default;
    virtual void step(time_point t, nanoseconds dt) = 0;
};

class scheduler {
public:
    scheduler() = default;
    ~scheduler() = default;

    void add_loop(const LoopSpec & spec);
    void run();
    void stop();
    void pause();
    void resume();

    bool override_clock(shared_ptr<IClock>);

private:
    //abstract implementation to hide details
    struct Impl;
    unique_ptr<Impl> impl_;
};
