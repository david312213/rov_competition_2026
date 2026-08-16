# 由真实 MAVLink 消息生成的机器人遥测。
# valid_* 为 false 时，对应数值不得用于裁判展示或控制决策。

builtin_interfaces/Time stamp

bool valid_depth
float32 depth_m

bool valid_attitude
float32 roll_deg
float32 pitch_deg
float32 yaw_deg

bool valid_battery_voltage
float32 battery_voltage_v

bool valid_battery_current
float32 battery_current_a

bool valid_battery_remaining
int16 battery_remaining_pct

bool valid_heartbeat
bool armed
string flight_mode
uint8 system_status

float32 heartbeat_age_s
float32 message_age_s
