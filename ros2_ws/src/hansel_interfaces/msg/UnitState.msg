uint8 ROLE_UNKNOWN=0
uint8 ROLE_HEAD=1
uint8 ROLE_REAR=2

uint8 INITIALIZING=0
uint8 STOPPED=1
uint8 ACTIVE=2
uint8 DETACHING=3
uint8 RELAY_ASSUMED=4
uint8 ESTOP=5
uint8 FAULT=6

builtin_interfaces/Time stamp
string unit_id
uint8 role
uint8 operation_state
bool active_for_drive
builtin_interfaces/Time last_command_stamp
string status_message

