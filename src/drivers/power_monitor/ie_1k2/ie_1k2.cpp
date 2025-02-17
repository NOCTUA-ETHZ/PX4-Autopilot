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

// Define your buffer size
static constexpr size_t RECEIVE_BUFFER_SIZE = 256;

IE_1K2::IE_1K2() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	snprintf(_port, sizeof(_port), "/dev/ttyS1");

	// Optional constructor debug:
	PX4_DEBUG("IE_1K2 constructor: Using UART port: %s", _port);
}

IE_1K2::~IE_1K2()
{
	if (_uart_fd >= 0) {
		::close(_uart_fd);
	}
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int IE_1K2::init()
{
	if (_initialized) {
		PX4_WARN("IE_1K2 already initialized");
		return PX4_OK;
	}

	_uart_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart_fd < 0) {
		PX4_ERR("Failed to open port %s", _port);
		return PX4_ERROR;
	}

	if (!configureUart()) {
		PX4_ERR("configureUart() failed");
		::close(_uart_fd);
		_uart_fd = -1;
		return PX4_ERROR;
	}

	// Schedule periodic execution every 100 ms
	ScheduleOnInterval(100000);
	_initialized = true;

	// --- Debug print here to confirm init done and scheduling started ---
	PX4_INFO("Hello from IE_1K2 init, build 2025-02-18! Driver init complete1.");

	return PX4_OK;
}

bool IE_1K2::configureUart()
{
	struct termios uart_config{};

	if (tcgetattr(_uart_fd, &uart_config) < 0) {
		PX4_ERR("Failed to get UART config on %s", _port);
		return false;
	}

	// Example: 9600 baud
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

	return true;
}

void IE_1K2::processData(const uint8_t *buffer, size_t length)
{
	char local_buf[256 + 1] = {};

	if (length > 256) {
		length = 256;
	}
	memcpy(local_buf, buffer, length);
	local_buf[length] = '\0';

	PX4_INFO("processData(): Received text: '%s'", local_buf);

	ie_fuel_cell_s report{};
	report.timestamp = hrt_absolute_time();

	// Example: <48,0.91,49.7,3968,2921,0,2,4,0,RECONDITION NEEDED,68>
	int parsed = sscanf(local_buf,
		"<%f,%f,%f,%f,%f,%ld,%f,%ld,%ld,%31[^,],%ld>",
		&report.tank_pressure,        // float  -> %f
		&report.regulated_pressure,   // float  -> %f
		&report.battery_voltage,      // float  -> %f
		&report.output_power,         // float  -> %f
		&report.spm_power_draw,       // float  -> %f
		&report.unit_in_fault,        // int32_t-> %ld
		&report.battery_output_power, // float  -> %f
		&report.psu_state,            // int32_t-> %ld
		&report.main_error_code,      // int32_t-> %ld
		report.info_string,           // char[32] -> %31[^,]
		&report.checksum              // int32_t-> %ld
	);

	if (parsed == 11) {
		_last_read_time = report.timestamp;
		_fuel_cell_pub.publish(report);

		PX4_INFO("Parsed OK: tank=%.2f, regulated=%.2f, info=%s, checksum=%ld",
		         (double)report.tank_pressure,
		         (double)report.regulated_pressure,
		         report.info_string,
		         report.checksum);

	} else {
		_parse_errors++;
		PX4_WARN("Failed to parse data (parsed %d fields)", parsed);
	}
}

void IE_1K2::Run()
{
	// --- Debug print to confirm Run() is firing ---
	PX4_ERR("IE_1K2::Run() called. Reading from UART...");

	if (_uart_fd < 0) {
		PX4_WARN("UART not open in Run()");
		return;
	}

	static enum class ParseMode {
		NONE,
		ANGLE,
		BRACKET
	} parse_mode = ParseMode::NONE;

	static uint8_t angle_buffer[RECEIVE_BUFFER_SIZE];
	static size_t angle_length = 0;

	static uint8_t bracket_buffer[RECEIVE_BUFFER_SIZE];
	static size_t bracket_length = 0;

	uint8_t byte;
	while (::read(_uart_fd, &byte, 1) > 0) {

		switch (parse_mode) {

		case ParseMode::NONE: {
			if (byte == '<') {
				parse_mode = ParseMode::ANGLE;
				angle_length = 0;
				angle_buffer[angle_length++] = byte;

			} else if (byte == '[') {
				parse_mode = ParseMode::BRACKET;
				bracket_length = 0;
				bracket_buffer[bracket_length++] = byte;
			}
		} break;

		case ParseMode::ANGLE: {
			if (angle_length < RECEIVE_BUFFER_SIZE) {
				angle_buffer[angle_length++] = byte;

				if (byte == '>') {
					parse_mode = ParseMode::NONE;
					angle_buffer[angle_length] = '\0';
					PX4_INFO("End-of-angle-bracket message: length=%zu", angle_length);

					processData(angle_buffer, angle_length);
					angle_length = 0;
				}

			} else {
				PX4_WARN("Angle buffer overflow. Clearing buffer.");
				_rx_errors++;
				parse_mode = ParseMode::NONE;
				angle_length = 0;
			}
		} break;

		case ParseMode::BRACKET: {
			if (bracket_length < RECEIVE_BUFFER_SIZE) {
				bracket_buffer[bracket_length++] = byte;

				if (byte == ']') {
					parse_mode = ParseMode::NONE;
					bracket_buffer[bracket_length] = '\0';
					PX4_INFO("Bracket line: %s", bracket_buffer);
					bracket_length = 0;
				}

			} else {
				PX4_WARN("Bracket buffer overflow. Clearing buffer.");
				_rx_errors++;
				parse_mode = ParseMode::NONE;
				bracket_length = 0;
			}
		} break;
		}
	}

	// If no data received for 1 second, print a warning
	if (hrt_elapsed_time(&_last_read_time) > 1000000) {
		PX4_WARN("No data received for over 1 second");
	}
}

int IE_1K2::print_status()
{
	IE_1K2 *instance = _object.load();

	if (instance == nullptr) {
		PX4_INFO("Driver not running (_object is null)");
	} else {
		PX4_INFO("Driver running (instance at %p)", (void *)instance);
		PX4_INFO("rx_errors: %" PRIu32 ", parse_errors: %" PRIu32,
		         _rx_errors, _parse_errors);
	}
	return 0;
}

int IE_1K2::task_spawn(int argc, char *argv[])
{
	IE_1K2 *instance = new IE_1K2();
	if (instance == nullptr) {
		PX4_ERR("Allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);

	if (instance->init() != PX4_OK) {
		PX4_ERR("Driver init failed");
		delete instance;
		_object.store(nullptr);
		return PX4_ERROR;
	}

	_task_id = task_id_is_work_queue;
	return PX4_OK;
}

extern "C" __EXPORT int ie_1k2_main(int argc, char *argv[])
{
	return IE_1K2::main(argc, argv);
}

