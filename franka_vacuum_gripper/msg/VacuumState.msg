Header header
bool in_control_range   # Vacuum value within setpoint area.
bool part_detached      # The part has been dtached after a suction cycle.
bool part_present       # Vacuum is over H2 and not yet under h2. For more information check the cobot-pump manual.
string device_status    # Current vacuum gripper device status
                          # Green:  Device is working optimally
                          # Yellow: Device is working but there are warnings
                          # Orange: Device is working but there are severe warnings
                          # Red:    Device is not working properly
uint16 actual_power     # Current vacuum gripper actual power [%]
uint16 vacuum           # Current system vacuum [mbar]
