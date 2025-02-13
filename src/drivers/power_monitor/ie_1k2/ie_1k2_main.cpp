/****************************************************************************
 * ie_1k2_main.cpp
 *
 * PX4 driver for the Intelligent Energy 1.2kW fuel cell
 ****************************************************************************/

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/ie_fuel_cell.h>

#include <string.h>
#include <termios.h>
#include <fcntl.h>

using namespace time_literals;

class IE_1k2 : public ModuleBase<IE_1k2>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    IE_1k2();
    ~IE_1k2() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

private:
    void Run() override;
    bool configure_uart();
    void process_data();

    /* Parameters */
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::IE_PORT>) _param_port,
        (ParamInt<px4::params::IE_BAUD>) _param_baud
    );

    int _uart_fd{-1};
    char _port[20];
    
    uORB::Publication<ie_fuel_cell_s> _fuel_cell_pub{ORB_ID(ie_fuel_cell)};
    
    static constexpr size_t RECV_BUFFER_SIZE = 256;
    uint8_t _receive_buffer[RECV_BUFFER_SIZE];
    size_t _receive_len{0};

    bool _initialized{false};
    hrt_abstime _last_read_time{0};
    
    uint32_t _rx_errors{0};
    uint32_t _parse_errors{0};
};

IE_1k2::IE_1k2() :
    ModuleParams(nullptr),
    ScheduledWorkItem("ie_1k2", px4::wq_configurations::hp_default)
{
    sprintf(_port, "/dev/ttyS%d", _param_port.get());
}

IE_1k2::~IE_1k2()
{
    if (_uart_fd >= 0) {
        ::close(_uart_fd);
    }
}

bool IE_1k2::init()
{
    if (_initialized) {
        return true;
    }

    // Open UART
    _uart_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    
    if (_uart_fd < 0) {
        PX4_ERR("Failed to open port %s", _port);
        return false;
    }

    if (!configure_uart()) {
        ::close(_uart_fd);
        _uart_fd = -1;
        return false;
    }

    // Schedule regular runs
    ScheduleOnInterval(100_ms); // 10Hz update rate

    _initialized = true;
    return true;
}

bool IE_1k2::configure_uart()
{
    termios uart_config{};

    if (tcgetattr(_uart_fd, &uart_config) < 0) {
        PX4_ERR("Failed to get UART config on %s", _port);
        return false;
    }

    // Set baud rate
    speed_t speed;
    switch (_param_baud.get()) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            PX4_ERR("Unsupported baud rate: %d", _param_baud.get());
            return false;
    }

    cfsetispeed(&uart_config, speed);
    cfsetospeed(&uart_config, speed);

    uart_config.c_cflag |= (CLOCAL | CREAD);    // Enable receiver, ignore modem ctrl lines
    uart_config.c_cflag &= ~CSIZE;
    uart_config.c_cflag |= CS8;                 // 8 data bits
    uart_config.c_cflag &= ~PARENB;             // No parity
    uart_config.c_cflag &= ~CSTOPB;             // 1 stop bit
    uart_config.c_cflag &= ~CRTSCTS;            // No hardware flow control

    uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
    uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG | TOSTOP);

    if (tcsetattr(_uart_fd, TCSANOW, &uart_config) < 0) {
        PX4_ERR("Failed to set UART config on %s", _port);
        return false;
    }

    return true;
}

void IE_1k2::Run()
{
    if (_uart_fd < 0) {
        return;
    }

    // Read available data
    uint8_t byte;
    while (::read(_uart_fd, &byte, 1) > 0) {
        if (_receive_len < RECV_BUFFER_SIZE) {
            _receive_buffer[_receive_len++] = byte;
            
            // Check for complete message
            if (byte == '>') {
                process_data();
                _receive_len = 0;
            }
        } else {
            // Buffer overflow
            _receive_len = 0;
            _rx_errors++;
        }
    }

    // Check for timeout
    if (hrt_elapsed_time(&_last_read_time) > 1_s) {
        PX4_WARN("No data received for 1 second");
    }
}

void IE_1k2::process_data()
{
    if (_receive_len < 3 || _receive_buffer[0] != '<') {
        _parse_errors++;
        return;
    }

    // Null terminate for string operations
    _receive_buffer[_receive_len] = '\0';

    ie_fuel_cell_s report{};
    report.timestamp = hrt_absolute_time();

    // Parse the message using format from IE documentation
    int parsed = sscanf((char*)_receive_buffer, 
        "<%f,%f,%f,%f,%f,%" SCNd32 ",%f,%" SCNd32 ",%" SCNd32 ",%31[^,],%*d>",
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
    } else {
        _parse_errors++;
        PX4_DEBUG("Parse failed: got %d of 10 fields", parsed);
    }
}

int IE_1k2::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
Driver for the Intelligent Energy 1.2kW Fuel Cell power system.

### Examples
CLI usage example:
$ ie_1k2 start
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("ie_1k2", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int IE_1k2::custom_command(int argc, char *argv[])
{
    return print_usage("Unrecognized command");
}

int IE_1k2::task_spawn(int argc, char *argv[])
{
    auto *instance = new IE_1k2();

    if (!instance) {
        PX4_ERR("Allocation failed");
        return -1;
    }

    if (!instance->init()) {
        delete instance;
        return -1;
    }

    _object.store(instance);
    _task_id = task_id_is_work_queue;

    return 0;
}

extern "C" __EXPORT int ie_1k2_main(int argc, char *argv[])
{
    return IE_1k2::main(argc, argv);
}
