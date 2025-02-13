uint64 timestamp          # Required: system time when the message was received
float32 tank_pressure      # Percent hydrogen remaining
float32 regulated_pressure # Regulated output pressure (Bar)
float32 battery_voltage    # Hybrid battery voltage
float32 output_power       # Combined system output power (W)
float32 spm_power_draw     # Fuel cell power draw (W)
float32 battery_output_power # Hybrid battery output power (W)
int32   psu_state         # Fuel cell power state: 0=Off, 1=Starting, 2=Running, 3=Stopping, 4=Sleep
int32   main_error_code   # Fuel cell error codes (See manufacturer documentation)
int32   subcode           # Fuel cell detailed error subcodes
char[32] info_string      # Additional info string
