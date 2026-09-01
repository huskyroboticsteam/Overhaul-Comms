uint8 DEGREES=1
uint8 METERS=2

uint64 session_id
uint64 sequence
string<=32 joint
float64 position
# Physical joints use DEGREES; ikUp and ikForward use METERS.
uint8 unit
