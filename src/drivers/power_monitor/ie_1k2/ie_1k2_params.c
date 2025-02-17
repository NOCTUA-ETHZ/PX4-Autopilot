// File: src/drivers/power_monitor/ie_1k2/ie_1k2_params.c

/**
 * IE 1.2kW Fuel Cell Serial Port
 *
 * @min 0
 * @max 6
 * @group Sensors
 * @reboot_required true
 * @value 0 UART 0
 * @value 1 UART 1
 * @value 2 UART 2
 * @value 3 UART 3
 * @value 4 UART 4
 * @value 5 UART 5
 * @value 6 UART 6
 */
PARAM_DEFINE_INT32(IE_PORT, 2);

/**
 * IE 1.2kW Fuel Cell Baud Rate
 *
 * @min 9600
 * @max 115200
 * @group Sensors
 * @reboot_required true
 * @value 9600 9600 baud
 * @value 19200 19200 baud
 * @value 38400 38400 baud
 * @value 57600 57600 baud
 * @value 115200 115200 baud
 */
PARAM_DEFINE_INT32(IE_BAUD, 9600);


