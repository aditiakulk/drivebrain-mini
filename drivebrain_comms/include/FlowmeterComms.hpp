// declares the flowmeter driver that reads GPIO pulses on its own threads and shares the latest measurements
#pragma once
#include <linux/gpio.h>
#include <DriverBus.hpp>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

// One detected pulse
struct FlowmeterPulse {
    // kernel timestamp of the rising edge (monotonic ns)
    std::uint64_t timestamp = 0;
};

// Flowmeter measurements
struct FlowmeterSnapshot {
    // total pulses since start
    std::uint64_t pulse = 0;
    // timestamp of the newest pulse
    std::uint64_t last_pulse = 0;
    // pulses per second from the last two pulses
    double frequency = 0.0;
};

// no pulse for this long -> report frequency as 0 (1 second)
constexpr std::uint64_t NO_FLOW_TIMEOUT_NS = 1'000'000'000ULL;

class FlowmeterComms {
public:
    // only saves the config, start() opens the pin
    FlowmeterComms(const std::string& chip, unsigned int pin);
    // calls stop() so threads are always joined
    ~FlowmeterComms();

    // Prevents copying the driver
    // (it owns threads and a file descriptor)
    FlowmeterComms(const FlowmeterComms&) = delete;
    FlowmeterComms& operator=(const FlowmeterComms&) = delete;

    // opens the pin and starts both threads
    void start();
    // stops and joins both threads, closes the pin
    void stop();
    // returns a copy of the measurements, safe to call from any thread
    FlowmeterSnapshot latest() const;
// All the private variables
private:
    // reader thread: kernel events -> queue
    void read_pulses();
    // processor thread: queue -> measurements
    void process_pulses();

    // GPIO configuration
    std::string chip;
    unsigned int pin;
    // fd for the requested line, -1 when closed
    int gpio = -1;

    // Worker threads and shutdown
    // atomic since every thread reads it
    std::atomic<bool> stopping{false};
    // Bug fix: lets the main loop detect a reader failure instead of reporting old data
    std::atomic<bool> failed{false};
    std::thread reader;
    std::thread processor;
    // reader is finished, guarded by queue.mtx so the processor can't miss the wakeup
    bool reader_done = false;

    // Incoming pulses
    // deque + mutex + condition variable from DriverBus.hpp
    core::ThreadSafeDeque<FlowmeterPulse> queue;

    // Shared measurements
    // mutable so latest() can lock it while being const
    mutable std::mutex data_mutex;
    FlowmeterSnapshot data;
};
