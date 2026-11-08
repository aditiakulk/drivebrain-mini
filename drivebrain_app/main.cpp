// connects the driver to logger so the program can read/report measuremeants
#include <FlowmeterComms.hpp>
#include <FoxgloveServer.hpp>
#include <MCAPLogger.hpp>
#include <drivebrain_msgs.pb.h>

#include <chrono>
#include <csignal>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <spdlog/spdlog.h>

volatile std::sig_atomic_t stopping = 0;

void sig_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        stopping = 1;
    }
}

// sends the same measurements to both loggers
void log_data(const FlowmeterSnapshot& data) {
    auto message = std::make_shared<drivebrain_msgs::FlowmeterData>();

    message->set_pulse(data.pulse);
    message->set_last_pulse(data.last_pulse);
    message->set_frequency(data.frequency);

    if (core::MCAPLogger::instance().log_msg(message) != 0) {
        throw std::runtime_error("Could not queue flowmeter recording");
    }

    core::FoxgloveServer::instance().send_live_telem_msg(message);
}
// lets you choose between the chip and line
int main(int argc, char* argv[]) {
    if (argc != 3) {
        spdlog::error(
            "Usage: {} /dev/gpiochipN LINE_OFFSET",
            argv[0]
        );
        return 1;
    }

    std::signal(SIGINT, sig_handler);
    std::signal(SIGTERM, sig_handler);

    int result = 0;

    try {
        // reads the GPIO configuration
        std::string chip = argv[1];
        std::string input = argv[2];

        if (input.empty() ||
            input.find_first_not_of("0123456789") != std::string::npos) {
            throw std::invalid_argument(
                "GPIO line offset must be a nonnegative integer"
            );
        }

        unsigned long value = std::stoul(input);

        if (value > std::numeric_limits<unsigned int>::max()) {
            throw std::out_of_range("GPIO line offset is too large");
        }

        unsigned int pin = static_cast<unsigned int>(value);

        // starts recording and live stuff
        std::filesystem::create_directories("/home/nixos/recordings");

        core::MCAPLogger::create(mcap::McapWriterOptions(""));

        if (core::MCAPLogger::instance().open_new_mcap() != 0) {
            throw std::runtime_error("Could not open MCAP recording");
        }

        core::MCAPLogger::instance().init_logging();
        core::FoxgloveServer::create();

        // starts the flowmeter workers
        FlowmeterComms flowmeter(chip, pin);
        flowmeter.start();

        spdlog::info(
            "Reading {} line {}; Foxglove port 5555",
            chip,
            pin
        );

        auto interval = std::chrono::milliseconds(20);
        auto next_tick = std::chrono::steady_clock::now();

        while (!stopping) {
            log_data(flowmeter.latest());

            next_tick += interval;

            auto now = std::chrono::steady_clock::now();

            if (now > next_tick) {
                next_tick = now;
            }

            std::this_thread::sleep_until(next_tick);
        }

        // finishes queued pulses and records the final measurements
        flowmeter.stop();
        log_data(flowmeter.latest());

        spdlog::info("Flowmeter workers stopped");
    } catch (const std::exception& error) {
        spdlog::error("mini-drivebrain: {}", error.what());
        result = 1;
    }

    // destroys the loggers after the flowmeter workers stop
    core::FoxgloveServer::destroy();
    core::MCAPLogger::destroy();

    spdlog::info("Logging stopped");

    return result;
}