/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ie_1k2.hpp"

#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>


// Protocol constants
static constexpr uint8_t START_FRAME_CHAR = '<';
static constexpr uint8_t END_FRAME_CHAR = '>';
static constexpr size_t MAX_FRAME_SIZE = 256;

IE_1K2::IE_1K2(const char *port, uint8_t rotation) :
    ScheduledWorkItem(DRIVER_NAME, px4::serial_port_to_wq(port)),
    _fuel_cell_pub(nullptr)
{
    PX4_INFO("IE_1K2 constructor - start");

    // Store the port name
    strncpy(_port, port, sizeof(_port) - 1);
    _port[sizeof(_port) - 1] = '\0';
    PX4_INFO("Port stored: %s", _port);

    // Initialize performance counters
_sample_perf = perf_alloc(PC_ELAPSED, DRIVER_NAME ": read");
_comms_errors = perf_alloc(PC_COUNT, DRIVER_NAME ": comm errors");
_parse_errors = perf_alloc(PC_COUNT, DRIVER_NAME ": parse errors");


    PX4_INFO("IE_1K2 constructor - complete");
}

IE_1K2::~IE_1K2()
{
    stop();

    perf_free(_sample_perf);
    perf_free(_comms_errors);
    perf_free(_parse_errors);
}

int IE_1K2::init()
{
    PX4_INFO("IE_1K2::init() - start");

    // Initialize publisher if not already done
    if (!_fuel_cell_pub.advertised()) {
        PX4_INFO("Creating publisher...");
        if (!_fuel_cell_pub.advertised()) {
            PX4_ERR("Failed to create publisher");
            return PX4_ERROR;
        }
    }

    PX4_INFO("Starting driver...");
    start();

    PX4_INFO("IE_1K2::init() - complete");
    return PX4_OK;
}

int IE_1K2::open_serial_port(const speed_t speed)
{
	PX4_INFO("IE_1K2::open_serial_port()");
    // File descriptor already initialized?
    if (_uart_fd > 0) {
        return PX4_OK;
    }

    // Configure port flags for read/write, non-controlling, non-blocking
    int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

    // Open the serial port
    _uart_fd = ::open(_port, flags);

    if (_uart_fd < 0) {
        PX4_ERR("Failed to open port %s", _port);
        return PX4_ERROR;
    }

    // Configure UART
    struct termios uart_config {};

    // Get current configuration
    if (tcgetattr(_uart_fd, &uart_config) < 0) {
        PX4_ERR("Failed to get UART config");
        ::close(_uart_fd);
        return PX4_ERROR;
    }

    // Set baud rate
    if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
        PX4_ERR("Failed to set UART speed");
        ::close(_uart_fd);
        return PX4_ERROR;
    }

    // Set data bits, stop bits, parity
    uart_config.c_cflag |= (CLOCAL | CREAD);
    uart_config.c_cflag &= ~CSIZE;
    uart_config.c_cflag |= CS8;
    uart_config.c_cflag &= ~(PARENB | PARODD);
    uart_config.c_cflag &= ~CSTOPB;
    uart_config.c_cflag &= ~CRTSCTS;

    // Set raw input/output
    uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
    uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG | TOSTOP);

    // Apply configuration
    if (tcsetattr(_uart_fd, TCSANOW, &uart_config) < 0) {
        PX4_ERR("Failed to set UART attributes");
        ::close(_uart_fd);
        return PX4_ERROR;
    }

    return PX4_OK;
}

int IE_1K2::collect()
{
	PX4_INFO("IE_1K2::collect()");
    perf_begin(_sample_perf);

    uint8_t buffer[MAX_FRAME_SIZE];
    ssize_t bytes_read = ::read(_uart_fd, buffer, sizeof(buffer));

    if (bytes_read < 0) {
        if (errno != EAGAIN) {
            PX4_ERR("read error: %d", errno);
            perf_count(_comms_errors);
            // Add this line to close and reopen the port on error
            stop();
            start();
        }
        perf_end(_sample_perf);
        return bytes_read;
    }

    // Process received data
    for (int i = 0; i < bytes_read; i++) {
        if (process_byte(buffer[i]) == PX4_OK) {
            // Successfully parsed a complete frame
            ie_fuel_cell_s report{};
            if (parse_message(_parse_buffer, _parse_buffer_len, report)) {
                report.timestamp = hrt_absolute_time();
                _fuel_cell_pub.publish(report);
                _last_read_time = report.timestamp;
            } else {
                perf_count(_parse_errors);
            }
        }
    }

    perf_end(_sample_perf);
    return PX4_OK;
}

int IE_1K2::process_byte(uint8_t byte)
{
    switch (_parse_state) {
        case ParseState::WAITING_START:
            if (byte == START_FRAME_CHAR) {
                _parse_state = ParseState::IN_FRAME;
                _parse_buffer_len = 0;
                _parse_buffer[_parse_buffer_len++] = byte;
            }
            break;

        case ParseState::IN_FRAME:
            if (byte == END_FRAME_CHAR) {
                _parse_buffer[_parse_buffer_len++] = byte;
                _parse_state = ParseState::WAITING_START;
                return PX4_OK; // Complete frame received
            } else if (_parse_buffer_len < MAX_FRAME_SIZE - 1) {
                _parse_buffer[_parse_buffer_len++] = byte;
            } else {
                _parse_state = ParseState::WAITING_START;
                perf_count(_parse_errors);
            }
            break;
    }

    return PX4_ERROR; // Frame not complete yet
}

bool IE_1K2::parse_message(const uint8_t *buffer, size_t len, ie_fuel_cell_s &report)
{
    // Ensure null termination for string operations
    char msg[MAX_FRAME_SIZE + 1];
    memcpy(msg, buffer, len);
    msg[len] = '\0';

    // Parse the message using sscanf
    int result = sscanf(msg,
        "<%f,%f,%f,%f,%f,%ld,%f,%ld,%ld,%31[^,],%ld>",
        &report.tank_pressure,
        &report.regulated_pressure,
        &report.battery_voltage,
        &report.output_power,
        &report.spm_power_draw,
        &report.unit_in_fault,
        &report.battery_output_power,
        &report.psu_state,
        &report.main_error_code,
        report.info_string,
        &report.checksum
    );

    return (result == 11);
}

void IE_1K2::start()
{
	PX4_INFO("IE_1K2::start()");
    // Configure the serial port
    if (open_serial_port(B9600) != PX4_OK) {
        PX4_ERR("Failed to open serial port");
        return;
    }

    // Schedule the driver to run on an interval
    ScheduleOnInterval(1000000); // 1000ms interval

    PX4_INFO("IE_1K2 driver started");
}

void IE_1K2::stop()
{
    ScheduleClear();

    if (_uart_fd >= 0) {
        ::close(_uart_fd);
        _uart_fd = -1;
    }
}

void IE_1K2::Run()
{
	PX4_INFO("IE_1K2::Run()");
    // Ensure the serial port is open
    if (_uart_fd < 0) {
        if (open_serial_port(B9600) != PX4_OK) {
            return;
        }
    }

    // Perform collection
    collect();

    // Check for data timeout
    if (hrt_elapsed_time(&_last_read_time) > 1000000) { // 1 second
        PX4_WARN("No data received for over 1 second");
    }
}

void IE_1K2::print_info()
{
    PX4_INFO("Device port: %s", _port);
    perf_print_counter(_sample_perf);
    perf_print_counter(_comms_errors);
    perf_print_counter(_parse_errors);
}

int IE_1K2::task_spawn(int argc, char *argv[])
{
    PX4_INFO("IE_1K2::task_spawn() - start");

    // Default to ttyS3
    const char *port = "/dev/ttyS3";
    PX4_INFO("Port set to: %s", port);

    // Check if port specified
    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    PX4_INFO("Parsing arguments...");
    while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
            case 'd':
                port = myoptarg;
                PX4_INFO("New port specified: %s", port);
                break;
        }
    }

    PX4_INFO("Creating instance...");
    IE_1K2 *instance = new IE_1K2(port);

    if (instance == nullptr) {
        PX4_ERR("Allocation failed");
        return PX4_ERROR;
    }

    PX4_INFO("Initializing instance...");
    if (instance->init() != PX4_OK) {
        PX4_ERR("Instance initialization failed");
        delete instance;
        return PX4_ERROR;
    }

    PX4_INFO("Storing instance...");
    _object.store(instance);
    _task_id = task_id_is_work_queue;

    PX4_INFO("Task spawn completed successfully");
    return PX4_OK;
}

IE_1K2 *IE_1K2::instantiate(int argc, char *argv[])
{
	PX4_INFO("IE_1K2::instantiate()");
    // Default to ttyS1
    const char *port = "/dev/ttyS3";

    // Check if port specified
    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
            case 'd':
                port = myoptarg;
                break;
        }
    }

    return new IE_1K2(port);
}

int IE_1K2::custom_command(int argc, char *argv[])
{
    return print_usage("Unknown command");
}

int IE_1K2::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
Driver for the IE_1K2 Fuel Cell power system.

### Examples
It is typically started with:
$ ie_1k2 start -d /dev/ttyS1
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("ie_1k2", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", "<file:dev>", "Serial port", true);
    PRINT_MODULE_USAGE_COMMAND("stop");
    PRINT_MODULE_USAGE_COMMAND("status");

    return 0;
}

extern "C" __EXPORT int ie_1k2_main(int argc, char *argv[])
{
    return IE_1K2::main(argc, argv);
}
