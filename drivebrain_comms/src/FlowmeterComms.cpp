// reads flowmeter pulses and processes them in separate threads
#include <FlowmeterComms.hpp>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <deque>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <system_error>
#include <sys/ioctl.h>
#include <unistd.h>

// only saves the config, nothing opens until start()
FlowmeterComms::FlowmeterComms(const std::string& chip, unsigned int pin)
    : chip(chip), pin(pin) {}

// makes sure threads are joined even if stop() was never called
FlowmeterComms::~FlowmeterComms() {
    stop();
}

// opens GPIO and starts the threads
void FlowmeterComms::start() {
    // can't start twice
    if (gpio != -1) {
        throw std::logic_error("Flowmeter is already started");
    }

    // opens the chip, only needed to request the line
    int device = open(chip.c_str(), O_RDONLY | O_CLOEXEC);

    if (device < 0) {
        throw std::system_error(
            errno, std::generic_category(), "Open " + chip
        );
    }

    // asks for one input line that reports rising edges
    // kernel buffers up to 64 events if we fall behind
    gpio_v2_line_request request{};
    request.offsets[0] = pin;
    request.num_lines = 1;
    request.event_buffer_size = 64;
    request.config.flags =
        GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_EDGE_RISING;

    // name that shows up in gpioinfo as the line's owner
    std::snprintf(
        request.consumer,
        sizeof(request.consumer),
        "mini-drivebrain"
    );

    // kernel gives back a new fd just for this line
    // saves errno before close() can change it
    int result = ioctl(device, GPIO_V2_GET_LINE_IOCTL, &request);
    int error = errno;
    close(device);

    if (result < 0) {
        throw std::system_error(
            error, std::generic_category(), "Request GPIO line"
        );
    }

    gpio = request.fd;

    try {
        // non blocking so read() never hangs the reader
        int flags = fcntl(gpio, F_GETFL);

        if (flags < 0 ||
            fcntl(gpio, F_SETFL, flags | O_NONBLOCK) < 0) {
            throw std::system_error(
                errno, std::generic_category(), "Configure GPIO"
            );
        }

        // resets state in case this driver was started before
        stopping = false;
        failed = false;

        {
            std::lock_guard<std::mutex> lock(queue.mtx);
            reader_done = false;
            queue.deque.clear();
        }

        {
            std::lock_guard<std::mutex> lock(data_mutex);
            data = {};
        }

        // processor first so it's already waiting when pulses come in
        processor = std::thread(
            &FlowmeterComms::process_pulses, this
        );

        reader = std::thread(
            &FlowmeterComms::read_pulses, this
        );
    } catch (...) {
        // setup failed partway: wakes the processor, joins it, closes the pin, rethrows
        stopping = true;

        {
            std::lock_guard<std::mutex> lock(queue.mtx);
            reader_done = true;
        }

        queue.cv.notify_all();

        if (processor.joinable()) {
            processor.join();
        }

        close(gpio);
        gpio = -1;
        throw;
    }
}

// stops the threads and closes GPIO
void FlowmeterComms::stop() {
    // reader sees this within 100ms (poll timeout)
    stopping = true;

    // reader exits first and wakes the processor on its way out
    if (reader.joinable()) {
        reader.join();
    }

    // processor finishes leftover pulses then exits
    if (processor.joinable()) {
        processor.join();
    }

    // closes the pin last since the reader was using it
    if (gpio != -1) {
        close(gpio);
        gpio = -1;
    }
}

// safely copies the latest measurements for the main loop
FlowmeterSnapshot FlowmeterComms::latest() const {
    // reports a dead reader instead of logging old data forever
    if (failed) {
        throw std::runtime_error(
            "Flowmeter reader failed; check its error message"
        );
    }

    FlowmeterSnapshot snapshot;

    // holds the lock just long enough to copy
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        snapshot = data;
    }

    // frequency only updates when a pulse arrives, so once flow stops it would
    // stay stuck at the last value. if no pulse for a while, report 0 instead.
    // kernel event timestamps use CLOCK_MONOTONIC, same clock as steady_clock
    auto now = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count()
    );

    if (snapshot.pulse > 0 && now - snapshot.last_pulse > NO_FLOW_TIMEOUT_NS) {
        snapshot.frequency = 0.0;
    }

    return snapshot;
}

// reads incoming pulses and adds them to the queue
void FlowmeterComms::read_pulses() {
    try {
        while (!stopping) {
            // sleeps until there's an event or 100ms passes, then rechecks stopping
            pollfd input{gpio, POLLIN, 0};
            int result = poll(&input, 1, 100);

            if (result < 0) {
                // a signal interrupted poll, just try again
                if (errno == EINTR) {
                    continue;
                }

                throw std::system_error(
                    errno, std::generic_category(), "Poll GPIO"
                );
            }

            if (stopping) {
                break;
            }

            // timed out, nothing happened
            if (result == 0) {
                continue;
            }

            // fd broke or closed
            if (input.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                throw std::runtime_error(
                    "GPIO event connection failed"
                );
            }

            if (!(input.revents & POLLIN)) {
                continue;
            }

            // reads up to 16 queued edge events at once
            gpio_v2_line_event events[16]{};
            ssize_t bytes = read(gpio, events, sizeof(events));

            if (bytes < 0) {
                // EAGAIN = nothing left to read (non blocking)
                if (errno == EINTR || errno == EAGAIN) {
                    continue;
                }

                throw std::system_error(
                    errno, std::generic_category(), "Read GPIO"
                );
            }

            // kernel always hands back whole events
            if (bytes == 0 || bytes % sizeof(events[0]) != 0) {
                throw std::runtime_error(
                    "Invalid GPIO event data"
                );
            }

            // pushes the whole batch under one lock
            {
                std::lock_guard<std::mutex> lock(queue.mtx);

                auto count =
                    static_cast<std::size_t>(bytes) / sizeof(events[0]);

                for (std::size_t i = 0; i < count; ++i) {
                    if (events[i].id == GPIO_V2_LINE_EVENT_RISING_EDGE) {
                        queue.deque.push_back({
                            events[i].timestamp_ns
                        });
                    }
                }
            }

            // wakes the processor after releasing the lock
            queue.cv.notify_one();
        }
    } catch (const std::exception& error) {
        // marks the failure so latest() can report it
        std::cerr << "Flowmeter: " << error.what() << '\n';
        failed = true;
        stopping = true;
    }

    // tells the processor no more pulses will be added
    // set under the lock so the processor can't miss the notify
    {
        std::lock_guard<std::mutex> lock(queue.mtx);
        reader_done = true;
    }

    queue.cv.notify_all();
}

// waits for queued pulses and updates the measurements
void FlowmeterComms::process_pulses() {
    while (true) {
        std::deque<FlowmeterPulse> pulses;

        {
            std::unique_lock<std::mutex> lock(queue.mtx);

            // sleeps until there are pulses or the reader is done (no busy loop)
            // the condition also handles spurious wakeups
            queue.cv.wait(lock, [this] {
                return reader_done || !queue.deque.empty();
            });

            // only exits once everything left has been processed
            if (reader_done && queue.deque.empty()) {
                break;
            }

            // takes all pulses at once so the reader isn't blocked during the math
            pulses.swap(queue.deque);
        }

        // queue lock is released here, now updates the shared measurements
        std::lock_guard<std::mutex> lock(data_mutex);

        for (const auto& pulse : pulses) {
            // frequency = 1 / time between pulses (ns -> Hz)
            if (data.pulse > 0 &&
                pulse.timestamp > data.last_pulse) {
                data.frequency =
                    1e9 / (pulse.timestamp - data.last_pulse);
            }

            ++data.pulse;
            data.last_pulse = pulse.timestamp;
        }
    }
}
