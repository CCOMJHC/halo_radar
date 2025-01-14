# Message type to relay radar settings and how to control them.

uint8 CONTROL_TYPE_FLOAT           = 0
uint8 CONTROL_TYPE_FLOAT_WITH_AUTO = 1
uint8 CONTROL_TYPE_ENUM            = 2

string   name          # Name of the control
string   value         # Current value to display
string   label         # Label used for user interfaces

uint8    type          # Type of value, see above CONTROL_TYPE constants

float32  min_value     # Minimum possible value
float32  max_value     # Maximum possible value

string[] enums         # Possible enums
