uint64 timestamp           # Required: system time when the message was received

float32 tank_pressure      # (1) Percent hydrogen remaining
float32 regulated_pressure # (2) Regulated (stack inlet) pressure, Bar
float32 battery_voltage    # (3) Hybrid battery voltage
float32 output_power       # (4) System output power
float32 spm_power_draw     # (5) Fuel cell power draw

int32   unit_in_fault      # (6) Unit in fault (for parallel config?), 0 or 1?

float32 battery_output_power # (7) Hybrid battery power

int32   psu_state          # (8) 0=Off, 1=Starting, 2=Running, etc.
int32   main_error_code    # (9) Main error code
char[32] info_string       # (10) Info string, e.g. "RECONDITION NEEDED"
int32   checksum           # (11) Checksum or final integer

