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
    std::uint64_t timestamp = 0;
};

// Flowmeter measurements
struct FlowmeterSnapshot {
    std::uint64_t pulse = 0;
    std::uint64_t last_pulse = 0;
    double frequency = 0.0;
};

class FlowmeterComms {
public:
    FlowmeterComms(const std::string& chip, unsigned int pin);
    ~FlowmeterComms();

    // Prevents copying the driver
    FlowmeterComms(const FlowmeterComms&) = delete;
    FlowmeterComms& operator=(const FlowmeterComms&) = delete;

    void start();
    void stop();
    FlowmeterSnapshot latest() const;
// All the private variables
private:
    void read_pulses();
    void process_pulses();

    // GPIO configuration
    std::string chip;
    unsigned int pin;
    int gpio = -1;

    // Worker threads and shutdown
    std::atomic<bool> stopping{false};
    // Bug fix: lets the main loop detect a reader failure instead of reporting old data
    std::atomic<bool> failed{false};
    std::thread reader;
    std::thread processor;
    bool reader_done = false;

    // Incoming pulses
    core::ThreadSafeDeque<FlowmeterPulse> queue;

    // Shared measurements
    mutable std::mutex data_mutex;
    FlowmeterSnapshot data;
};