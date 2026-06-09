#pragma once

#include <Arduino.h>

// ==================== 旧 ESP32Robot.ino 继承引脚 ====================

// RS485 接收引脚，ESP32-S3 GPIO16 连接 TTL-RS485 模块的 RX/RO 相关信号。
#define RS485_RX_PIN 16

// RS485 发送引脚，ESP32-S3 GPIO17 连接 TTL-RS485 模块的 TX/DI 相关信号。
#define RS485_TX_PIN 17

// PCA9685 I2C 数据线 SDA，必须使用旧项目规定的 GPIO11。
#define PCA9685_SDA_PIN 11

// PCA9685 I2C 时钟线 SCL，必须使用旧项目规定的 GPIO12。
#define PCA9685_SCL_PIN 12

// 激光模块控制引脚，默认常亮；B2 自动工位模式下闪烁。
#define LASER_PIN 15

// B2 自动工位模式下激光闪烁间隔，单位毫秒。
constexpr uint32_t LASER_BLINK_INTERVAL_MS = 250;

// Laser blink interval while an automatic station action is running.
constexpr uint32_t LASER_AUTO_RUNNING_BLINK_INTERVAL_MS = 80;
constexpr uint32_t LASER_RC_CONNECTED_BLINK_INTERVAL_MS = 80;
constexpr uint32_t LASER_RC_CONNECTED_BLINK_DURATION_MS = 3000;

// Chassis drive limits used by M1-M4 wheel speed commands.
constexpr int CHASSIS_MAX_SPEED_RPM_LIMIT = 200;
constexpr uint8_t CHASSIS_ACCEL_WHEEL_LIMIT = 188;

// S4 actual-angle targets used after M5 reaches the lift/top position in auto station mode.
constexpr int AUTO_PRE_CLAMP_S4_AFTER_LIFT = 80;
constexpr int AUTO_TURNTABLE_S4_AFTER_LIFT = 170;
constexpr int AUTO_PRE_DROP_S4_AFTER_LIFT = 80;

// ==================== MC7RE / SBUS 遥控器输入 ====================

// 遥控接收机串口输入引脚，MC7RE 的 M.BUS/SBUS 信号线接 GPIO18。
#define RC_RX_PIN 18

// 遥控接收机串口发送引脚，本项目只读取接收机，GPIO19 预留不用也可以。
#define RC_TX_PIN 19

// SBUS 常见为反相信号：1 表示按反相串口读取，0 表示普通串口读取。
#define RC_SERIAL_INVERTED 1

// SBUS/M.BUS 串口波特率，标准 SBUS 通常为 100000。
#define RC_BAUD_RATE 100000

// SBUS 串口格式，标准 SBUS 通常为 8 数据位、偶校验、2 停止位。
#define RC_SERIAL_CONFIG SERIAL_8E2

// ==================== 遥控器通道索引 ====================

// CH1 在 SBUS 通道数组中的索引，默认 0；通常对应右摇杆左右。
#define RC_CH1_INDEX 0

// CH2 在 SBUS 通道数组中的索引，默认 1；通常对应右摇杆上下。
#define RC_CH2_INDEX 1

// CH3 在 SBUS 通道数组中的索引，默认 2；通常对应左摇杆上下。
#define RC_CH3_INDEX 2

// CH4 在 SBUS 通道数组中的索引，默认 3；通常对应左摇杆左右。
#define RC_CH4_INDEX 3

// CH5 在 SBUS 通道数组中的索引，默认 4；用于 A/B 大模式切换。
#define RC_CH5_INDEX 4

// CH6 在 SBUS 通道数组中的索引，默认 5；用于普通/特殊子模式切换。
#define RC_CH6_INDEX 5

// ==================== 遥控器数值范围与失控判断 ====================

// SBUS 原始最小值，标准常见范围约为 172~1811。
constexpr int RC_RAW_MIN = 172;

// SBUS 原始最大值，标准常见范围约为 172~1811。
constexpr int RC_RAW_MAX = 1811;

// 归一化后的遥控通道最小值，对应摇杆或开关一端。
constexpr int RC_OUT_MIN = 1000;

// 归一化后的遥控通道中位值，模式判断也以 1500 为分界。
constexpr int RC_OUT_MID = 1500;

// 归一化后的遥控通道最大值，对应摇杆或开关另一端。
constexpr int RC_OUT_MAX = 2000;

// 摇杆死区，通道值距离 1500 小于等于该值时视为 0；死区外按遥控器返回值线性换算速度百分比。
constexpr int RC_DEADZONE = 60;

// 遥控器超时时间，超过该毫秒数没有有效 SBUS 帧就触发 failsafe。
constexpr uint32_t RC_TIMEOUT_MS = 300;

// ==================== 底盘速度与轮电机参数 ====================

// 底盘上层速度命令最大百分比，遥控器返回值会线性映射到 -100~100。
constexpr int WHEEL_CMD_MAX = 100;

// B1 底盘微调模式的最大速度命令。
// 普通底盘模式满摇杆为 100；B1 满摇杆只到该值，实现“大行程、小速度”微调。
// 如果 B1 仍然太快，减小该值；如果车完全不动，确认它大于 MIN_EFFECTIVE_CMD。
constexpr int FINE_WHEEL_CMD_MAX = 12;

// M1~M4 轮电机最大转速，速度百分比会线性映射到 0~MAX_SPEED_RPM。
constexpr int MAX_SPEED_RPM = 250;

// 最小有效速度命令，绝对值小于该值时强制当作 0，避免低速抖动。
constexpr int MIN_EFFECTIVE_CMD = 3;

// M1~M4 轮电机速度模式加速度参数，继承旧项目设置。


constexpr uint8_t ACCEL_WHEEL = 250;

// 底盘横移方向修正：1 保持当前方向，-1 反转 CH1 横移方向。
#define CHASSIS_X_SIGN -1

// 底盘前后方向修正：1 保持当前方向，-1 反转 CH2 前后方向。
#define CHASSIS_Y_SIGN 1

// 底盘原地转向方向修正：1 保持当前方向，-1 反转 CH4 转向方向。
#define CHASSIS_W_SIGN -1

// M1 单轮方向修正：1 保持当前方向，-1 反转 M1。
#define M1_SIGN 1

// M2 单轮方向修正：1 保持当前方向，-1 反转 M2。
#define M2_SIGN 1

// M3 单轮方向修正：1 保持当前方向，-1 反转 M3。
#define M3_SIGN 1

// M4 单轮方向修正：1 保持当前方向，-1 反转 M4。
#define M4_SIGN 1

// ==================== M5 / M6 绝对位置模式参数 ====================

// M5 机械臂升降电机的位置模式加速度参数，继承旧项目设置。
constexpr uint8_t ACCEL_MOTOR5 = 250;

// M6 物料盘旋转电机的位置模式加速度参数，继承旧项目设置。
constexpr uint8_t ACCEL_MOTOR6 = 40;

// M5 默认位置命令速度参数；自动工位会显式使用 POSITION_SPEED_M5_AUTO。
constexpr uint16_t POSITION_SPEED_M5 = 1000;

// M5 自动工位时使用的运动速度参数，只影响 B2 自动工位模式。
// 如果自动工位中 M5 太快，就调小这个值；如果太慢，就调大这个值。
constexpr uint16_t POSITION_SPEED_M5_AUTO = 2000;

// M6 绝对位置命令使用的运动速度参数，只影响物料盘电机。
constexpr uint16_t POSITION_SPEED_M6 = 60;

// 位置命令模式字节，0x01 表示绝对位置模式。
constexpr uint8_t POSITION_MODE_ABSOLUTE = 0x01;

// 位置命令同步字节，0x00 表示不启用多机同步。
constexpr uint8_t POSITION_SYNC_DISABLED = 0x00;

// M5 机械臂升降最小目标脉冲数；现在 motor5Pos 直接就是发给电机的位置脉冲。
constexpr int32_t M5_PULSE_MIN = 0;

// M5 机械臂升降最大目标脉冲数；程序会把 M5 限制在 M5_PULSE_MIN~M5_PULSE_MAX。
constexpr int32_t M5_PULSE_MAX = 158000;




// M5 每次手动升降增加或减少的脉冲数，CH3 持续打到底时每 50ms 改一次。
constexpr int32_t M5_STEP = 1000;

// M5 方向修正：1 保持当前 CH3 上推位置减小，-1 反转升降方向。
#define M5_DIR_SIGN 1

// M6 物料盘最小目标脉冲数，负数表示一个方向旋转。
constexpr int32_t M6_MIN = -100000;

// M6 物料盘最大目标脉冲数，正数表示另一个方向旋转。
constexpr int32_t M6_MAX = 100000;

// M6 物料盘回中目标脉冲数；现在不再换算角度，直接作为脉冲目标发送。
constexpr int32_t M6_CENTER = 1800;

// M6 每次左右转动增加或减少的脉冲数。
constexpr int32_t M6_STEP = 533;

// 转台连贯宏中 M6 每次自动转盘步进脉冲数；方向反了可改成 -M6_STEP。
constexpr int32_t AUTO_TURNTABLE_M6_STEP = M6_STEP;

// ==================== PCA9685 与舵机参数 ====================

// PCA9685 I2C 地址，旧项目固定为 0x40。
constexpr uint8_t PCA9685_ADDRESS = 0x40;

// 舵机 PWM 频率，普通舵机通常使用 50Hz。
constexpr uint16_t SERVO_PWM_FREQ = 50;

// 舵机 0 度对应的 PCA9685 脉宽计数，继承旧项目设置。
constexpr int SERVOMIN = 102;

// 舵机 180 度对应的 PCA9685 脉宽计数，继承旧项目设置。
constexpr int SERVOMAX = 512;

// S1 云台/机械臂旋转舵机上电默认逻辑角度。
constexpr int SERVO_DEFAULT_S1 = 180;

// S2 夹爪一侧舵机旧项目上电默认角度，当前启动后会由夹爪打开角覆盖。
constexpr int SERVO_DEFAULT_S2 = 0;

// S3 夹爪另一侧舵机旧项目上电默认角度，当前启动后会由夹爪打开角覆盖。
constexpr int SERVO_DEFAULT_S3 = 0;

// S4 机械臂前后舵机上电默认实际角度。
constexpr int SERVO_DEFAULT_S4 = 180;

// S4 机械臂前后舵机实际角度范围。
constexpr int SERVO_S4_MIN = 0;
constexpr int SERVO_S4_MAX = 180;

// S1/S4 手动模式每次增量调整的角度步长。
constexpr int SERVO_STEP = 5;

// 夹爪打开角度，S2/S3 默认都写这个角度。
constexpr int GRIPPER_OPEN_ANGLE = 60;

// 转台放料时的临时半开角度，先松到 120，再在回顶后完全打开到 60。
constexpr int GRIPPER_TURNTABLE_RELEASE_ANGLE = 120;

// 夹爪闭合角度，S2/S3 默认都写这个角度。
constexpr int GRIPPER_CLOSE_ANGLE = 170;

// 夹爪状态值：0 表示闭合。
constexpr uint8_t GRIPPER_STATE_CLOSED = 0;

// 夹爪状态值：1 表示张开。
constexpr uint8_t GRIPPER_STATE_OPEN = 1;

// S3 是否反向：0 表示 S3 与 S2 同角度，1 表示 S3 写入 180 - angle。
#define GRIPPER_S3_REVERSED 0

// ==================== 自动工位参数 ====================

// 自动工位切换时使用的安全高位；所有旋转 S1/S4 前都会先让 M5 到该位置。
// 当前 500 脉冲在 1000 脉冲以内，用于避免机械臂主轴旋转时碰撞。
constexpr int32_t HIGH_POS = 500;

// 夹取位 M5 目标脉冲数，沿用原预夹位数值。
constexpr int32_t PICKUP_MOTOR5 = 139860;

// M5 顶部归零位置。
constexpr int32_t M5_TOP_MOTOR5 = 0;

// 新预夹位 State 1 的 M5 目标脉冲数，比夹取位少 20000 脉冲。
constexpr int32_t PRE_CLAMP_MOTOR5 = PICKUP_MOTOR5 - 20000;

// 预夹位 State 1 的 S1 目标逻辑角度。
constexpr int PRE_CLAMP_SERVO0 = 0;

// 预夹位 State 1 的 S4 目标实际角度。
constexpr int PRE_CLAMP_SERVO3 = 180;

// 转台位 State 2 的 M5 目标脉冲数。
constexpr int32_t TURNTABLE_MOTOR5 = 76590;

// CH2-low automatic station macro: lift M5 first, then move to this target.
constexpr int32_t CH2_LOW_AUTO_LIFT_MOTOR5 = 0;
constexpr int32_t CH2_LOW_AUTO_TARGET_MOTOR5 = 42000;

// 转台位 State 2 的 S1 目标逻辑角度。
constexpr int TURNTABLE_SERVO0 = 180;

// 转台位 State 2 的 S4 目标实际角度：S4 内收。
constexpr int TURNTABLE_SERVO3 = 170;

// 预放位 State 3 的 M5 目标脉冲数。
constexpr int32_t PRE_DROP_MOTOR5 = 157000;

// 预放位 State 3 的 S1 目标逻辑角度。
constexpr int PRE_DROP_SERVO0 = 0;

// 预放位 State 3 的 S4 目标实际角度：S4 外展。
constexpr int PRE_DROP_SERVO3 = 60;

// S1/S4/夹爪手动控制更新周期，单位毫秒。
constexpr uint32_t SERVO_UPDATE_MS = 50;

// M5 手动升降旧连续控制预留周期；当前 M5 已改为边沿触发，此参数暂不参与控制。
constexpr uint32_t M5_UPDATE_MS = 50;

// 自动工位中 S1/S4 动作后的等待时间，给舵机留出到位时间。
constexpr uint32_t AUTO_SERVO_SETTLE_MS = 500;

// 自动工位中夹爪开合前后的固定等待时间。
constexpr uint32_t AUTO_GRIPPER_SETTLE_MS = 500;

// State 2 转台连贯宏：M5 到夹取位后的等待时间。
constexpr uint32_t TURNTABLE_PICKUP_MOVE_WAIT_MS = 2000;

// State 2 转台连贯宏：M5 到转台位后的等待时间。
constexpr uint32_t TURNTABLE_TO_TARGET_MOVE_WAIT_MS = 2000;

// State 3 合并预放宏：S4 内收到转台角后的等待时间。
constexpr uint32_t PRE_DROP_S4_TO_TURNTABLE_SETTLE_MS = 200;

// CH1 右推合并预放宏：S1 到转盘角度后，到启动 M5 下探之间的等待时间。
constexpr uint32_t PRE_DROP_S1_TO_CLAMP_SETTLE_MS = 1000;

// CH1 右推合并预放宏：M5 开始去转盘预夹位后，提前启动夹爪闭合的等待时间。
// 该值小于 AUTO_M5_MOVE_WAIT_MS 时，夹爪会在 M5 仍在接近 42000 的过程中提前闭合。
constexpr uint32_t PRE_DROP_M5_CLAMP_BEFORE_CLOSE_MS = 1500;

// CH1 右推合并预放宏：夹爪闭合完成后，M5 回顶部之前的额外等待时间。
constexpr uint32_t PRE_DROP_AFTER_CLOSE_WAIT_MS = 1000;

// 转台连贯宏中 M6 转盘单步后的等待时间。
constexpr uint32_t AUTO_M6_STEP_WAIT_MS = 500;

// 自动工位中 M5 每次位置命令后的固定等待时间，单位毫秒。
// 没有读取电机到位反馈时，用这个时间估算 M5 已经到位。
// 如果 S1/S4 旋转前 M5 还没完全回到 HIGH_POS，就调大这个值。
// 如果 B2 自动动作反应太慢，就调小这个值。
constexpr uint32_t AUTO_M5_MOVE_WAIT_MS = 3000;

// ==================== 摇杆边沿触发阈值 ====================

// 低端触发阈值，通道值小于该值时认为摇杆打到低端。
constexpr int EDGE_LOW_THRESHOLD = 1200;

// 高端触发阈值，通道值大于该值时认为摇杆打到高端。
constexpr int EDGE_HIGH_THRESHOLD = 1800;

// 自动工位中 CH3 上拨让 M5 归 0 的专用阈值；比普通高端阈值更高，避免误触发。
constexpr int AUTO_CH3_TOP_THRESHOLD = 1950;

// 边沿触发重新武装的中位下限，摇杆必须回到该值以上才允许再次触发低端。
constexpr int EDGE_CENTER_LOW = 1350;

// 边沿触发重新武装的中位上限，摇杆必须回到该值以下才允许再次触发高端。
constexpr int EDGE_CENTER_HIGH = 1650;

// ==================== 串口调试输出 ====================

// 调试信息打印周期，单位毫秒；数值越小串口刷新越快。
constexpr uint32_t DEBUG_PRINT_MS = 250;
