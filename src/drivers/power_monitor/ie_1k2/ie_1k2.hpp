#pragma once

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/ie_fuel_cell.h>
#include <sys/types.h>
#include <termios.h>
#include <px4_platform_common/getopt.h>

class IE_1K2 : public ModuleBase<IE_1K2>, public px4::ScheduledWorkItem
{
public:
    IE_1K2(const char *port, uint8_t rotation = 0);
    ~IE_1K2() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static IE_1K2 *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    int init();
    void print_info();

    // Module name constant
#define DRIVER_NAME "ie_1k2"

private:
    // Serial port configuration
    static constexpr uint32_t IE_1K2_DEFAULT_BAUDRATE = 9600;
    static constexpr uint32_t IE_1K2_DEFAULT_UPDATE_RATE = 10; // 100 Hz

    // Main loop functions
    void Run() override;
    void start();
    void stop();
    int collect();

    // Serial port handling
    int open_serial_port(const speed_t speed = B9600);

    // Message parsing
    enum class ParseState {
        WAITING_START,
        IN_FRAME
    };

    int process_byte(uint8_t byte);
    bool parse_message(const uint8_t *buffer, size_t len, ie_fuel_cell_s &report);

    // File descriptor
    int _uart_fd{-1};
    char _port[20] {};

    // Parser state
    ParseState _parse_state{ParseState::WAITING_START};
    uint8_t _parse_buffer[256];
    size_t _parse_buffer_len{0};

    // Publications
    //uORB::Publication<ie_fuel_cell_s> _fuel_cell_pub;
    uORB::Publication<ie_fuel_cell_s> _fuel_cell_pub{ORB_ID(ie_fuel_cell)};

    // Performance monitoring
    perf_counter_t _sample_perf{nullptr};
    perf_counter_t _comms_errors{nullptr};
    perf_counter_t _parse_errors{nullptr};

    // Timing
    hrt_abstime _last_read_time{0};
};
