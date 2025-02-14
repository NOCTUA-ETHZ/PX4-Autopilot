/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  (see header for full license text)
 *
 ****************************************************************************/

#include "ie_1k2.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

IE_1K2::IE_1K2() :
    // Use MODULE_NAME defined by the build system (see CMakeLists.txt)
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
    PX4_INFO("IE_1K2 constructor called");
    // For simplicity, we hardcode the UART port.
    // In a real driver you might retrieve this from parameters.
    snprintf(_port, sizeof(_port), "/dev/ttyS2");
    PX4_INFO("Using UART port: %s", _port);
}

IE_1K2::~IE_1K2()
{
    PX4_INFO("IE_1K2 destructor called");
    if (_uart_fd >= 0) {
        ::close(_uart_fd);
    }
    perf_free(_sample_perf);
    perf_free(_comms_errors);
}

int IE_1K2::init()
{
    PX4_INFO("IE_1K2::init() called");
    if (_initialized) {
        PX4_INFO("Already initialized");
        return PX4_OK;
    }

    _uart_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_uart_fd < 0) {
        PX4_ERR("Failed to open port %s", _port);
        return PX4_ERROR;
    }
    PX4_INFO("UART port %s opened, fd=%d", _port, _uart_fd);

    if (!configureUart()) {
        PX4_ERR("configureUart() failed");
        ::close(_uart_fd);
        _uart_fd = -1;
        return PX4_ERROR;
    }
    PX4_INFO("UART configured successfully");

    // Schedule periodic execution every 100ms (100000 microseconds).
    ScheduleOnInterval(100000);
    _initialized = true;
    PX4_INFO("IE_1K2 initialized and scheduled");
    return PX4_OK;
}

bool IE_1K2::configureUart()
{
    PX4_INFO("IE_1K2::configureUart() called");
    struct termios uart_config{};
    if (tcgetattr(_uart_fd, &uart_config) < 0) {
        PX4_ERR("Failed to get UART config on %s", _port);
        return false;
    }

    // Use 115200 baud by default (adjust as needed)
    speed_t speed = B9600;
    cfsetispeed(&uart_config, speed);
    cfsetospeed(&uart_config, speed);

    uart_config.c_cflag |= (CLOCAL | CREAD);
    uart_config.c_cflag &= ~CSIZE;
    uart_config.c_cflag |= CS8;
    uart_config.c_cflag &= ~PARENB;
    uart_config.c_cflag &= ~CSTOPB;
    uart_config.c_cflag &= ~CRTSCTS;

    uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
    uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG | TOSTOP);

    if (tcsetattr(_uart_fd, TCSANOW, &uart_config) < 0) {
        PX4_ERR("Failed to set UART config on %s", _port);
        return false;
    }

    PX4_INFO("UART config set on %s", _port);
    return true;
}

void IE_1K2::processData()
{
    PX4_INFO("processData() called, length=%zu", _receive_length);
    // We expect the data frame to start with '<'
    if (_receive_length < 3 || _receive_buffer[0] != '<') {
        _parse_errors++;
        PX4_WARN("Received data not valid: length=%zu, first char=%c", _receive_length, _receive_buffer[0]);
        return;
    }

    // Null-terminate the received data for sscanf.
    _receive_buffer[_receive_length] = '\0';

    ie_fuel_cell_s report{};
    report.timestamp = hrt_absolute_time();

    // Parse the string according to your protocol.
    int parsed = sscanf(reinterpret_cast<char*>(_receive_buffer),
           "<%f,%f,%f,%f,%f,%ld,%f,%ld,%ld,%31[^,],%*d>",
           &report.tank_pressure,
           &report.regulated_pressure,
           &report.battery_voltage,
           &report.output_power,
           &report.spm_power_draw,
           &report.psu_state,
           &report.battery_output_power,
           &report.main_error_code,
           &report.subcode,
           report.info_string);

    if (parsed == 10) {
        _last_read_time = report.timestamp;
        _fuel_cell_pub.publish(report);
        PX4_INFO("Data parsed successfully: tank_pressu");
    } else {
        _parse_errors++;
        PX4_WARN("Failed to parse data, parsed %d fields", parsed);
    }
}

void IE_1K2::Run()
{
PX4_INFO("Running..");
    if (_uart_fd < 0) {
        PX4_WARN("UART not open in Run()");
        return;
    }

    uint8_t byte;
    while (::read(_uart_fd, &byte, 1) > 0) {
        if (_receive_length < RECEIVE_BUFFER_SIZE) {
            _receive_buffer[_receive_length++] = byte;
            if (byte == '>') { // End-of-message marker.
                PX4_INFO("End-of-message marker received");
                processData();
                _receive_length = 0;
            }
        } else {
            _receive_length = 0;
            _rx_errors++;
            PX4_WARN("Receive buffer overflow, clearing buffer");
        }
    }

    // If no data received for 1 second, print a warning.
    if (hrt_elapsed_time(&_last_read_time) > 1000000) {
        PX4_WARN("No data received for 1 second");
    }
}

int IE_1K2::print_status()
{
    IE_1K2 *instance = _object.load();
    if (instance == nullptr) {
        PX4_INFO("Driver not running (_object is null)");
    } else {
        PX4_INFO("Driver running (instance at %p)", (void *)instance);
        PX4_INFO("rx_errors: %" PRIu32 ", parse_errors: %" PRIu32, _rx_errors, _parse_errors);
    }
    return 0;
}


int IE_1K2::task_spawn(int argc, char *argv[])
{
    PX4_INFO("IE_1K2::task_spawn() called");
    IE_1K2 *instance = new IE_1K2();
    if (instance == nullptr) {
        PX4_ERR("Allocation failed");
        return PX4_ERROR;
    }
    _object.store(instance);
    PX4_INFO("Instance stored at %p", (void *)_object.load());
    if (instance->init() != PX4_OK) {
        PX4_ERR("Driver init failed");
        delete instance;
        _object.store(nullptr);
        return PX4_ERROR;
    }
    // Set the task ID to indicate that the driver is running.
    _task_id = task_id_is_work_queue;
    PX4_INFO("All good! Driver is running. Task id: %d", _task_id);
    return PX4_OK;
}


extern "C" __EXPORT int ie_1k2_main(int argc, char *argv[])
{
    return IE_1K2::main(argc, argv);
}

