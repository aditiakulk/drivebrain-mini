// reads flowmeter pulses and processes them in separate threads
#include <FlowmeterComms.hpp>

#include <cerrno>
#include <cstdio>
#include <deque>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <system_error>
#include <sys/ioctl.h>
#include <unistd.h>

FlowmeterComms::FlowmeterComms(const std::string& chip, unsigned int pin)
    : chip(chip), pin(pin) {}

FlowmeterComms::~FlowmeterComms() {
    stop();
}

// opens GPIO and starts the threads
void FlowmeterComms::start() {
    if (gpio != -1) {
        throw std::logic_error("Flowmeter is already started");
    }

    int device = open(chip.c_str(), O_RDONLY | O_CLOEXEC);

    if (device < 0) {
        throw std::system_error(
            errno, std::generic_category(), "Open " + chip
        );
    }

    gpio_v2_line_request request{};
    request.offsets[0] = pin;
    request.num_lines = 1;
    request.event_buffer_size = 64;
    request.config.flags =
        GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_EDGE_RISING;

    std::snprintf(
        request.consumer,
        sizeof(request.consumer),
        "mini-drivebrain"
    );

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
        int flags = fcntl(gpio, F_GETFL);

        if (flags < 0 ||
            fcntl(gpio, F_SETFL, flags | O_NONBLOCK) < 0) {
            throw std::system_error(
                errno, std::generic_category(), "Configure GPIO"
            );
        }

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

        processor = std::thread(
            &FlowmeterComms::process_pulses, this
        );

        reader = std::thread(
            &FlowmeterComms::read_pulses, this
        );
    } catch (...) {
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
    stopping = true;

    if (reader.joinable()) {
        reader.join();
    }

    if (processor.joinable()) {
        processor.join();
    }

    if (gpio != -1) {
        close(gpio);
        gpio = -1;
    }
}

// safely copies the latest measurements for the main loop
FlowmeterSnapshot FlowmeterComms::latest() const {
    if (failed) {
        throw std::runtime_error(
            "Flowmeter reader failed; check its error message"
        );
    }

    std::lock_guard<std::mutex> lock(data_mutex);
    return data;
}

// reads incoming pulses and adds them to the queue
void FlowmeterComms::read_pulses() {
    try {
        while (!stopping) {
            pollfd input{gpio, POLLIN, 0};
            int result = poll(&input, 1, 100);

            if (result < 0) {
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

            if (result == 0) {
                continue;
            }

            if (input.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                throw std::runtime_error(
                    "GPIO event connection failed"
                );
            }

            if (!(input.revents & POLLIN)) {
                continue;
            }

            gpio_v2_line_event events[16]{};
            ssize_t bytes = read(gpio, events, sizeof(events));

            if (bytes < 0) {
                if (errno == EINTR || errno == EAGAIN) {
                    continue;
                }

                throw std::system_error(
                    errno, std::generic_category(), "Read GPIO"
                );
            }

            if (bytes == 0 || bytes % sizeof(events[0]) != 0) {
                throw std::runtime_error(
                    "Invalid GPIO event data"
                );
            }

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

            queue.cv.notify_one();
        }
    } catch (const std::exception& error) {
        std::cerr << "Flowmeter: " << error.what() << '\n';
        failed = true;
        stopping = true;
    }

    // tells the processor no more pulses will be added
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

            queue.cv.wait(lock, [this] {
                return reader_done || !queue.deque.empty();
            });

            if (reader_done && queue.deque.empty()) {
                break;
            }

            pulses.swap(queue.deque);
        }

        std::lock_guard<std::mutex> lock(data_mutex);

        for (const auto& pulse : pulses) {
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