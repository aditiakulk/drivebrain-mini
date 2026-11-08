// connects the flowmeter driver to the loggers so its measurements get recorded and streamed live
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

// set by the signal handler, checked by the main loop
// sig_atomic_t so a signal can't interrupt a half finished write
volatile std::sig_atomic_t stopping = 0;

// runs on ctrl-c (SIGINT) or systemctl stop (SIGTERM)
// only sets a flag since a handler can interrupt anything
void sig_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        stopping = 1;
    }
}

// sends the same measurements to both loggers
void log_data(const FlowmeterSnapshot& data) {
    // loggers take a shared_ptr to the base protobuf message
    auto message = std::make_shared<drivebrain_msgs::FlowmeterData>();

    // copies the snapshot into the protobuf
    message->set_pulse(data.pulse);
    message->set_last_pulse(data.last_pulse);
    message->set_frequency(data.frequency);

    // queues it for the mcap thread (returns -1 if logging stopped)
    if (core::MCAPLogger::instance().log_msg(message) != 0) {
        throw std::runtime_error("Could not queue flowmeter recording");
    }

    // sends it to anyone connected on foxglove
    core::FoxgloveServer::instance().send_live_telem_msg(message);
}

// default wiring, used when the binary is started with no arguments
// (the pi's mini-drivebrain.service runs /home/nixos/mini-drivebrain with no args)
// GPIO17 = physical pin 11. check the chip with `gpiodetect` on the pi
const char* DEFAULT_CHIP = "/dev/gpiochip0";
const char* DEFAULT_LINE = "17";

// lets you choose between the chip and line
int main(int argc, char* argv[]) {
    // argc counts the program name, so 0 args = 1 and 2 args = 3
    if (argc != 1 && argc != 3) {
        spdlog::error(
            "Usage: {} [/dev/gpiochipN LINE_OFFSET]",
            argv[0]
        );
        return 1;
    }

    // registers the handlers before anything starts
    std::signal(SIGINT, sig_handler);
    std::signal(SIGTERM, sig_handler);

    int result = 0;

    // everything is in here so cleanup at the bottom still runs if something throws
    try {
        // reads the GPIO configuration
        std::string chip = argc == 3 ? argv[1] : DEFAULT_CHIP;
        std::string input = argc == 3 ? argv[2] : DEFAULT_LINE;

        // checks digits ourselves since stoul accepts "-1" and "17abc"
        if (input.empty() ||
            input.find_first_not_of("0123456789") != std::string::npos) {
            throw std::invalid_argument(
                "GPIO line offset must be a nonnegative integer"
            );
        }

        unsigned long value = std::stoul(input);

        // line offsets are 32 bit in the kernel struct
        if (value > std::numeric_limits<unsigned int>::max()) {
            throw std::out_of_range("GPIO line offset is too large");
        }

        unsigned int pin = static_cast<unsigned int>(value);

        // starts recording and live stuff
        std::filesystem::create_directories("/home/nixos/recordings");

        // makes the logger singleton and opens the next N.mcap
        core::MCAPLogger::create(mcap::McapWriterOptions(""));

        if (core::MCAPLogger::instance().open_new_mcap() != 0) {
            throw std::runtime_error("Could not open MCAP recording");
        }

        // starts the mcap writer thread, then the websocket on port 5555
        core::MCAPLogger::instance().init_logging();
        core::FoxgloveServer::create();

        // starts the flowmeter workers
        // local to this block so its destructor joins the threads before the loggers are destroyed
        FlowmeterComms flowmeter(chip, pin);
        flowmeter.start();

        spdlog::info(
            "Reading {} line {}; Foxglove port 5555",
            chip,
            pin
        );

        // logs at a fixed 50 Hz
        auto interval = std::chrono::milliseconds(20);
        auto next_tick = std::chrono::steady_clock::now();

        while (!stopping) {
            // copies the latest measurements safely and logs them
            log_data(flowmeter.latest());

            // aims for fixed times instead of sleeping 20ms after the work, so it doesn't drift
            next_tick += interval;

            auto now = std::chrono::steady_clock::now();

            // if we fell behind, starts over from now instead of rushing to catch up
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
        // nonzero exit so systemd restarts it
        spdlog::error("mini-drivebrain: {}", error.what());
        result = 1;
    }

    // destroys the loggers after the flowmeter workers stop
    // the mcap logger writes what's left and closes the file
    core::FoxgloveServer::destroy();
    core::MCAPLogger::destroy();

    spdlog::info("Logging stopped");

    return result;
}
