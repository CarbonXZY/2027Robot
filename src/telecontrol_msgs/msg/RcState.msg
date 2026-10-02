std_msgs/Header header

# 摇杆，-1.0 ~ +1.0
float32 right_x
float32 right_y
float32 left_x
float32 left_y

# 滑块，-100.0 ~ +100.0
float32 s1

# 开关三档，取值见下方常量
uint8 sa
uint8 sb
uint8 sc
uint8 sd
uint8 se

# 链路状态
uint8 rssi
uint8 link_quality
int8  snr
bool  failsafe

uint8 LOW=0
uint8 MIDDLE=1
uint8 HIGH=2
