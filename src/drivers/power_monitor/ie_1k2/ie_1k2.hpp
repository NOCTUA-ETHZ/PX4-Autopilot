/**
 * @file IE_1K2.hpp
 *
 * A minimal driver for the IE Fuel Cell.
 */

#pragma once

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/ie_fuel_cell.h>
#include <termios.h>
#include <perf/perf_counter.h>

class IE_1K2 : public ModuleBase<IE_1K2>, public px4::ScheduledWorkItem
{
public:
	IE_1K2();
	virtual ~IE_1K2() override;

	/** Start the driver (called from the main entry point) */
	static int task_spawn(int argc, char *argv[]);

	/** Minimal custom command stub (not used) */
	static int custom_command(int argc, char *argv[]) { return 0; }

	/** Minimal usage text stub (not used) */
	static int print_usage(const char *reason = nullptr) { return 0; }

	/** Module status information */
	int print_status() override;

	/** Initialize the driver */
	int init();

protected:
	/** Called on the work queue at a fixed interval */
	void Run() override;

private:
	bool configureUart();
	void processData();

	// UART file descriptor and port string
	int _uart_fd{-1};
	char _port[20]{};

	// uORB publication for fuel cell data
	uORB::Publication<ie_fuel_cell_s> _fuel_cell_pub{ORB_ID(ie_fuel_cell)};

	// Reception buffer (assuming a message fits within 256 bytes)
	static constexpr size_t RECEIVE_BUFFER_SIZE = 256;
	uint8_t _receive_buffer[RECEIVE_BUFFER_SIZE]{};
	size_t _receive_length{0};

	bool _initialized{false};
	hrt_abstime _last_read_time{0};

	uint32_t _rx_errors{0};
	uint32_t _parse_errors{0};

	// Optional performance counters
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "ie_1k2 read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "ie_1k2 comms errors")};
};

