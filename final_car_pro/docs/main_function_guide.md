# main.cpp 函数说明与调参指南

这份文档专门解释 `src/main.cpp` 里每个函数的目的、调用关系和调参入口。读代码时建议配合 `include/config.h` 一起看：`main.cpp` 负责逻辑，`config.h` 集中放引脚、通道、速度、角度、工位等参数。

## 阅读顺序

1. 先看 `setup()` 和 `loop()`，理解程序启动和主循环。
2. 再看遥控器输入函数，确认 CH1~CH6 是怎样来的。
3. 然后看 RS485 电机函数和 PCA9685 舵机函数，理解底层硬件输出。
4. 最后看四个模式处理函数和自动工位状态机。

## 主要全局变量

### 硬件对象

| 变量 | 作用 | 调参相关 |
| ---- | ---- | -------- |
| `MotorSerial(2)` | ESP32-S3 的串口 2，用于 RS485 电机通信 | 引脚在 `RS485_RX_PIN` / `RS485_TX_PIN` |
| `RcSerial(1)` | ESP32-S3 的串口 1，用于读取 MC7RE SBUS/M.BUS | 引脚和反相在 `RC_RX_PIN` / `RC_SERIAL_INVERTED` |
| `pwm` | PCA9685 舵机驱动对象 | 地址在 `PCA9685_ADDRESS` |
| `LASER_PIN` | GPIO15 激光模块控制脚 | 除 A1 底盘模式外输出高电平；CH5/CH6 中位和 failsafe 输出低电平 |

### 遥控器状态

| 变量 | 作用 | 注意 |
| ---- | ---- | ---- |
| `rcCh[16]` | 归一化后的遥控器通道，范围 `1000~2000` | 代码主要使用 CH1~CH6 |
| `sbusRaw[16]` | SBUS 原始通道值 | 一般不直接调 |
| `lastRcFrameMs` | 最近一次收到有效 SBUS 帧的时间 | 用于 `RC_TIMEOUT_MS` 失控判断 |
| `rcLostFlag` | SBUS lost frame 标志 | 接收机置位或超时会触发失控 |
| `rcFailsafeFlag` | SBUS failsafe 标志 | 接收机置位会触发失控 |
| `failsafeTriggered` | 本机失控保护锁存标志 | 避免每个 loop 重复发急停 |

### 执行动作状态

| 变量 | 作用 | 调参相关 |
| ---- | ---- | -------- |
| `panAngle` | S1 云台当前逻辑角度 | 初值 `SERVO_DEFAULT_S1` |
| `armServoLogicalAngle` | S4 当前逻辑角度 | 实际写入为 `180 - armServoLogicalAngle` |
| `gripperAngle` | S2/S3 夹爪角度 | `GRIPPER_OPEN_ANGLE` / `GRIPPER_CLOSE_ANGLE` |
| `motor5Pos` | M5 当前目标脉冲数 | 范围 `M5_PULSE_MIN~M5_PULSE_MAX`，CH3 按住时每 `M5_UPDATE_MS` 改变一次 `M5_STEP` |
| `motor6Pos` | M6 当前目标脉冲数 | 步长 `M6_STEP`，回中 `M6_CENTER` |
| `lastWheelCmd[4]` | M1~M4 上一次速度命令 | 用于避免重复发送相同速度 |
| `chassisStopped` | 底盘是否已经停止 | 防止模式切换时重复发停止命令 |

### 自动工位状态

| 变量 | 作用 |
| ---- | ---- |
| `autoRunning` | 自动工位是否正在执行 |
| `autoPhase` | 当前自动工位阶段，如到高位、动舵机、到目标位 |
| `currentAutoState` | 当前工位，默认 `2`，即转台位 |
| `targetAutoState` | 目标工位 |
| `autoPhaseStartMs` | 当前阶段开始时间 |
| `autoPhaseDurationMs` | 当前阶段预计等待时间 |
| `autoNeedsFinalMove` | 是否需要先到 `HIGH_POS` 后再到目标 M5 位置 |

## 辅助结构和小函数

### `EdgeLatch`

- 作用：把“摇杆打到底”变成一次性触发，必须回中后才能再次触发。
- 什么时候被调用：物料盘模式和自动工位模式中使用。
- 输入/输出：`low(value)` 检测低端触发，`high(value)` 检测高端触发，返回 `true` 表示刚触发一次。
- 会影响哪些硬件：本身不直接影响硬件，只决定是否允许执行 M6 或自动工位动作。
- 调参相关：`EDGE_LOW_THRESHOLD`、`EDGE_HIGH_THRESHOLD`、`EDGE_CENTER_LOW`、`EDGE_CENTER_HIGH`。
- 注意事项：如果触发太容易，收窄阈值；如果打到底也不触发，放宽阈值。

### `clamp32(int32_t value, int32_t low, int32_t high)`

- 作用：把 32 位整数限制在指定范围内。
- 什么时候被调用：M5/M6 绝对位置命令中。
- 输入/输出：输入原始值、最小值、最大值，返回限位后的值。
- 会影响哪些硬件：间接保护 M5/M6 不越界。
- 调参相关：M5/M6 的最小最大位置常量。
- 注意事项：它只是软件限位，不能替代机械限位和急停。

### `channel(uint8_t indexMacro)`

- 作用：读取 `rcCh[]` 中某个通道的当前归一化值。
- 什么时候被调用：几乎所有模式判断和控制逻辑都会调用。
- 输入/输出：输入通道索引宏，例如 `RC_CH1_INDEX`；返回 `1000~2000`。
- 会影响哪些硬件：本身不影响硬件。
- 调参相关：如果通道对应不对，改 `RC_CH1_INDEX` 到 `RC_CH6_INDEX`。
- 注意事项：这里读的是“索引”，不是 CH 编号本身。

## 程序启动流程

### `setup()`

- 作用：完成整机初始化。
- 什么时候被调用：ESP32 上电或复位后由 Arduino 框架调用一次。
- 输入/输出：无参数，无返回值。
- 会影响哪些硬件：初始化 USB 串口、遥控器串口、PCA9685、RS485 电机，并设置舵机初始角度。
- 调参相关：启动时的舵机角度来自 `SERVO_DEFAULT_S1`、`SERVO_DEFAULT_S4_LOGICAL`、`GRIPPER_OPEN_ANGLE`。
- 注意事项：`setupRS485Motors()` 会使能全部 M1~M6，并只给 M1~M4 清零；M5/M6 不清零。

启动顺序：

```text
Serial.begin
rcCh 初始化为 1500
setupRcReceiver
setupPCA9685
setupRS485Motors
设置 S1、S2/S3、S4 初始角度
```

## 主循环流程

### `loop()`

- 作用：主控制循环，负责读取遥控器、处理失控保护、分发四种工作模式。
- 什么时候被调用：Arduino 框架不断循环调用。
- 输入/输出：无参数，无返回值。
- 会影响哪些硬件：间接控制所有电机和舵机。
- 调参相关：模式分界使用 `RC_OUT_MID = 1500`；失控时间使用 `RC_TIMEOUT_MS`。
- 注意事项：自动工位运行时，普通手动模式不会抢控制；只有 CH2 下拉可以取消自动动作。

主流程简化如下：

```text
readRcChannels
如果遥控器无效 -> handleFailsafe -> 返回
如果刚从 failsafe 恢复 -> 必须摇杆回中
updateAutoStateMachine
如果自动工位运行中 -> 只允许取消，不执行手动控制
根据 CH5/CH6 分发到 DRIVE / TRAY / ARM / AUTO
printDebugInfoPeriodically
```

## 遥控器 / SBUS 函数

### `setupRcReceiver()`

- 作用：初始化 MC7RE 接收机串口。
- 什么时候被调用：`setup()` 中。
- 输入/输出：无参数，无返回值。
- 会影响哪些硬件：打开 `RcSerial`，读取 GPIO18 的 SBUS/M.BUS 信号。
- 调参相关：`RC_BAUD_RATE`、`RC_SERIAL_CONFIG`、`RC_RX_PIN`、`RC_TX_PIN`、`RC_SERIAL_INVERTED`。
- 注意事项：如果串口一直没有数据，优先尝试切换 `RC_SERIAL_INVERTED`。

### `readRcChannels()`

- 作用：从串口持续读取 SBUS 字节，组 25 字节帧，解析通道，并判断是否失控。
- 什么时候被调用：每次 `loop()` 一开始。
- 输入/输出：无参数；返回 `true` 表示当前有可用且未失控的遥控器数据。
- 会影响哪些硬件：只读遥控器，不直接控制硬件。
- 调参相关：`RC_TIMEOUT_MS`。
- 注意事项：如果没有新帧，但还没超时，会继续认为信号有效；超时后会触发 failsafe。

### `parseSbusFrame(const uint8_t* frame)`

- 作用：把一帧 SBUS 数据拆成 16 个原始通道，并读取 lost/failsafe 标志。
- 什么时候被调用：`readRcChannels()` 收满 25 字节后。
- 输入/输出：输入 25 字节 SBUS 帧；返回 `true` 表示帧头正确并已解析。
- 会影响哪些硬件：只更新遥控器状态。
- 调参相关：一般不调；如果你的接收机不是标准 SBUS，需要替换这里。
- 注意事项：标准 SBUS 帧头是 `0x0F`，通道位拆包不要随便改。

### `normalizeRcValue(int raw)`

- 作用：把 SBUS 原始值映射到 `1000~2000`。
- 什么时候被调用：`parseSbusFrame()` 中每个通道解析后。
- 输入/输出：输入原始值，返回归一化通道值。
- 会影响哪些硬件：间接影响全部遥控器控制灵敏度。
- 调参相关：`RC_RAW_MIN`、`RC_RAW_MAX`、`RC_OUT_MIN`、`RC_OUT_MAX`。
- 注意事项：如果串口打印的通道最大最小不到 `1000/2000`，可以根据实测修改 raw 范围。

### `channelPercent(int value)`

- 作用：把 `1000~2000` 的摇杆值转换成 `-100~100` 的速度百分比。
- 什么时候被调用：底盘模式中计算 `vx`、`vy`、`w`。
- 输入/输出：输入通道值；返回百分比速度。
- 会影响哪些硬件：间接影响 M1~M4 底盘电机速度。
- 调参相关：`RC_DEADZONE`、`WHEEL_CMD_MAX`。
- 注意事项：死区内返回 0，用来避免摇杆中位抖动导致车慢慢动。

### `allSticksCentered()`

- 作用：判断 CH1~CH4 摇杆是否都回到中位附近。
- 什么时候被调用：failsafe 恢复逻辑中。
- 输入/输出：无参数；返回 `true` 表示四个摇杆都在死区内。
- 会影响哪些硬件：不直接控制硬件，但决定是否允许恢复控制。
- 调参相关：`RC_DEADZONE`。
- 注意事项：失控恢复后必须回中，这是安全保护，不建议取消。

## PCA9685 / 舵机函数

### `setupPCA9685()`

- 作用：初始化 I2C 和 PCA9685。
- 什么时候被调用：`setup()` 中。
- 输入/输出：无参数，无返回值。
- 会影响哪些硬件：PCA9685 舵机驱动板。
- 调参相关：`PCA9685_SDA_PIN`、`PCA9685_SCL_PIN`、`PCA9685_ADDRESS`、`SERVO_PWM_FREQ`。
- 注意事项：I2C 固定使用 GPIO11/GPIO12，和旧项目一致。

### `setServoRawChannel(int pcaChannel, int angle)`

- 作用：直接向 PCA9685 某个通道写入角度。
- 什么时候被调用：`setServoLogical()` 间接调用。
- 输入/输出：输入 PCA9685 通道和实际角度；无返回值。
- 会影响哪些硬件：对应 PCA9685 通道上的舵机。
- 调参相关：`SERVOMIN`、`SERVOMAX`。
- 注意事项：这是底层函数，不处理 S4 反向逻辑；平时优先调用 `setServoLogical()`。

### `setServoLogical(uint8_t servoId, int angle)`

- 作用：按 S1~S4 逻辑编号设置舵机角度。
- 什么时候被调用：启动、手动机械臂、自动工位、夹爪函数中。
- 输入/输出：`servoId` 为 1~4，`angle` 为逻辑角度；无返回值。
- 会影响哪些硬件：S1~S4 舵机。
- 调参相关：S4 逻辑方向、舵机默认角度、自动工位角度。
- 注意事项：S4 会自动执行 `实际角度 = 180 - 逻辑角度`；S2/S3 夹爪建议通过 `setGripperAngle()` 控制。

### `setGripperAngle(int angle)`

- 作用：同时控制 S2 和 S3 夹爪角度。
- 什么时候被调用：启动和机械臂手动模式中。
- 输入/输出：输入夹爪角度；无返回值。
- 会影响哪些硬件：S2/S3 夹爪舵机。
- 调参相关：`GRIPPER_OPEN_ANGLE`、`GRIPPER_CLOSE_ANGLE`、`GRIPPER_S3_REVERSED`。
- 注意事项：如果夹爪两个舵机需要一正一反，把 `GRIPPER_S3_REVERSED` 改为 `1`。

### `updateLaserForMode()`

- 作用：根据当前 CH5/CH6 模式切换 GPIO15 激光输出。
- 什么时候被调用：`loop()` 通过 failsafe 恢复检查后、自动状态机和模式分发前。
- 输入/输出：无参数，无返回值。
- 会影响哪些硬件：GPIO15 上连接的激光模块。
- 调参相关：激光引脚在 `LASER_PIN`；模式阈值使用 `RC_OUT_MID = 1500`。
- 注意事项：A2、B1、B2 输出高电平；A1 底盘模式输出低电平。CH5 或 CH6 在中位区间 `EDGE_CENTER_LOW~EDGE_CENTER_HIGH` 时强制输出低电平，failsafe 也会关激光。

## RS485 电机函数

### `setupRS485Motors()`

- 作用：初始化 RS485 串口，使能 M1~M6，并给 M1~M4 清零。
- 什么时候被调用：`setup()` 中。
- 输入/输出：无参数，无返回值。
- 会影响哪些硬件：全部 6 个闭环步进电机。
- 调参相关：`RS485_RX_PIN`、`RS485_TX_PIN` 固定为 GPIO16/GPIO17。
- 注意事项：M5/M6 不清零，保留驱动器自己的位置记忆；这是为了避免机械臂一上电乱跑。

### `sendMotorCommand(uint8_t addr, const uint8_t* cmd, size_t len)`

- 作用：RS485 底层发送函数，打印命令，写串口，flush，等待并打印返回值。
- 什么时候被调用：所有电机命令函数都会调用它。
- 输入/输出：输入电机地址、命令数组、命令长度；无返回值。
- 会影响哪些硬件：RS485 总线上的目标电机。
- 调参相关：发送后的短等待是 `delay(20)`，通常不需要改。
- 注意事项：会识别 `0x02` OK、`0xE2` 参数错误、`0xEE` 格式错误。

### `motorEnable(uint8_t idx)`

- 作用：使能指定电机。
- 什么时候被调用：`setupRS485Motors()` 中对 M1~M6 调用。
- 输入/输出：`idx` 是 0 索引，0 对应 M1，5 对应 M6；无返回值。
- 会影响哪些硬件：指定 RS485 电机。
- 调参相关：电机地址数组 `motorAddr[6]`。
- 注意事项：命令格式继承旧项目：`addr F3 AB 01 00 6B`。

### `motorStop(uint8_t idx)`

- 作用：停止指定电机。
- 什么时候被调用：failsafe、取消自动工位、急停、单电机停止时。
- 输入/输出：`idx` 是 0 索引；无返回值。
- 会影响哪些硬件：指定 RS485 电机。
- 调参相关：一般不调。
- 注意事项：命令格式继承旧项目：`addr FE 98 00 6B`。

### `setCurrentAsZero(uint8_t idx)`

- 作用：把指定电机当前位置设为零点。
- 什么时候被调用：`setupRS485Motors()` 中只对 M1~M4 调用。
- 输入/输出：`idx` 是 0 索引；无返回值。
- 会影响哪些硬件：指定 RS485 电机的位置零点。
- 调参相关：一般不调。
- 注意事项：不要轻易给 M5/M6 调用，否则机械臂/物料盘的绝对位置基准会改变。

### `setMotorSpeed(uint8_t idx, int cmdPercent)`

- 作用：控制 M1~M4 底盘轮速度。
- 什么时候被调用：`handleDriveMode()` 和 `stopChassisOnce()`。
- 输入/输出：`idx` 为 0~3；`cmdPercent` 为 `-100~100`；无返回值。
- 会影响哪些硬件：M1~M4 底盘电机。
- 调参相关：`WHEEL_CMD_MAX`、`MAX_SPEED_RPM`、`MIN_EFFECTIVE_CMD`、`ACCEL_WHEEL`、`M1_SIGN~M4_SIGN`。
- 注意事项：如果速度和上一次相同，不会重复发 RS485 命令，减少总线压力。

### `setMotorAbsPosition(uint8_t idx, int32_t targetPosition)`

- 作用：控制 M5/M6 到绝对位置。
- 什么时候被调用：M5 手动升降、M6 物料盘、自动工位。
- 输入/输出：`idx=4` 表示 M5，`idx=5` 表示 M6；`targetPosition` 是目标脉冲数；无返回值。
- 会影响哪些硬件：M5 机械臂升降或 M6 物料盘。
- 调参相关：M5 手动速度用 `POSITION_SPEED_M5`，B2 自动速度用 `POSITION_SPEED_M5_AUTO`，M5 加速度用 `ACCEL_MOTOR5`；M6 速度用 `POSITION_SPEED_M6`，加速度用 `ACCEL_MOTOR6`。
- 注意事项：M5/M6 现在都不再做上层逻辑值映射，变量值会直接作为目标脉冲数发送给电机；负数只用于决定方向，实际脉冲数取绝对值。

## 安全保护函数

### `emergencyStopAllMotors()`

- 作用：停止 M1~M6 全部电机，并清空底盘速度缓存。
- 什么时候被调用：刚进入 failsafe 时。
- 输入/输出：无参数，无返回值。
- 会影响哪些硬件：全部 RS485 电机。
- 调参相关：一般不调。
- 注意事项：这个函数会连续发 6 个停止命令，所以只在必要时调用。

### `stopChassisOnce()`

- 作用：只在需要时停止 M1~M4 底盘，避免重复发停止命令。
- 什么时候被调用：进入物料盘、机械臂手动、自动工位模式时。
- 输入/输出：无参数，无返回值。
- 会影响哪些硬件：M1~M4 底盘电机。
- 调参相关：一般不调。
- 注意事项：依赖 `chassisStopped` 判断是否已经停过。

### `handleFailsafe()`

- 作用：处理遥控器丢失后的急停动作。
- 什么时候被调用：`loop()` 发现 `readRcChannels()` 返回 false 时。
- 输入/输出：无参数，无返回值。
- 会影响哪些硬件：停止全部电机，取消自动工位。
- 调参相关：`RC_TIMEOUT_MS`、SBUS lost/failsafe 标志。
- 注意事项：使用 `failsafeTriggered` 锁存，只在刚失控时发一次急停，避免 RS485 总线风暴。

### `handleFailsafeAndRecovery()`

- 作用：处理遥控器信号恢复后的安全解锁。
- 什么时候被调用：`loop()` 中遥控器数据恢复后。
- 输入/输出：返回 `true` 表示允许继续控制；返回 `false` 表示还要等待摇杆回中。
- 会影响哪些硬件：不直接控制硬件。
- 调参相关：`RC_DEADZONE`。
- 注意事项：必须 CH1~CH4 全部回中才会打印 `FAILSAFE CLEARED` 并恢复控制。

## 模式处理函数

### `handleDriveMode()`

- 作用：普通底盘模式，读取 CH1/CH2/CH4，计算 M1~M4 混控速度。
- 什么时候被调用：`CH5 < 1500 && CH6 < 1500`。
- 输入/输出：无参数，无返回值。
- 会影响哪些硬件：M1~M4 底盘电机。
- 调参相关：`CHASSIS_X_SIGN`、`CHASSIS_Y_SIGN`、`CHASSIS_W_SIGN`、`M1_SIGN~M4_SIGN`、`WHEEL_CMD_MAX`。
- 注意事项：输出会自动归一化，保证四个轮子的速度不超过 `-100~100`。

混控公式：

```cpp
m1 = vy - vx + w;
m2 = -vy + vx + w;
m3 = -vy - vx + w;
m4 = vy + vx + w;
```

### `handleTrayMode()`

- 作用：物料盘模式，CH1 左右控制 M6 一步一步转，CH2 下拉回中。
- 什么时候被调用：`CH5 < 1500 && CH6 > 1500`。
- 输入/输出：无参数，无返回值。
- 会影响哪些硬件：M6 物料盘，进入该模式会先停止底盘。
- 调参相关：`M6_STEP`、`M6_CENTER`、边沿触发阈值。
- 注意事项：使用 `EdgeLatch`，必须摇杆回中后才能再次触发一步。

### `handleArmManualMode()`

- 作用：机械臂手动模式，控制 S1、S4、M5 和夹爪。
- 什么时候被调用：`CH5 > 1500 && CH6 < 1500`。
- 输入/输出：无参数，无返回值。
- 会影响哪些硬件：S1、S2、S3、S4、M5，并先停止底盘。
- 调参相关：`SERVO_STEP`、`SERVO_UPDATE_MS`、`M5_STEP`、`M5_UPDATE_MS`、夹爪角度。
- 注意事项：S1/S4/夹爪按 50ms 节奏更新；M5 也按 `M5_UPDATE_MS` 连续更新，CH3 持续打到底会持续升降。

### `updateM5ByChannel(int ch3)`

- 作用：根据 CH3 控制 M5 升降，按住 CH3 时按周期连续发送新位置。
- 什么时候被调用：`handleArmManualMode()` 中。
- 输入/输出：输入 CH3 当前值；无返回值。
- 会影响哪些硬件：M5 机械臂升降电机。
- 调参相关：`M5_STEP`、`M5_DIR_SIGN`、`M5_UPDATE_MS`、`EDGE_LOW_THRESHOLD`、`EDGE_HIGH_THRESHOLD`。
- 注意事项：`motor5Pos` 现在直接是目标脉冲数。CH3 上推时目标脉冲数减小；CH3 下拉时目标脉冲数增大。按住不放会每 `M5_UPDATE_MS` 毫秒发送一次。

### `handleArmAutoMode()`

- 作用：自动工位模式，接收工位选择和取消指令。
- 什么时候被调用：`CH5 > 1500 && CH6 > 1500`。
- 输入/输出：无参数，无返回值。
- 会影响哪些硬件：可能启动 M5/S1/S4 自动动作，并先停止底盘。
- 调参相关：自动工位参数和边沿触发阈值。
- 注意事项：自动工位运行中不会接受新的工位选择，只接受 CH2 下拉取消。

## 自动工位函数

### `startAutoState(uint8_t state)`

- 作用：启动一个自动工位切换流程。
- 什么时候被调用：`handleArmAutoMode()` 检测到 CH1/CH2 边沿触发时。
- 输入/输出：输入目标状态 `1/2/3`；无返回值。
- 会影响哪些硬件：立即可能发送 M5 到 `HIGH_POS` 或目标位置的命令。
- 调参相关：`HIGH_POS`、`PRE_CLAMP_*`、`TURNTABLE_*`、`PRE_DROP_*`。
- 注意事项：如果目标状态等于当前状态，只打印提示不动作；如果已有自动流程在跑，也不会重复启动。

### `cancelAutoMacro(bool stopMotors)`

- 作用：取消自动工位流程。
- 什么时候被调用：failsafe、自动模式 CH2 下拉取消、自动运行中取消。
- 输入/输出：`stopMotors=true` 时会额外停止 M5/M6；无返回值。
- 会影响哪些硬件：可选停止 M5/M6。
- 调参相关：一般不调。
- 注意事项：failsafe 中已经调用全电机急停，所以传 `false`，避免重复停止。

### `updateAutoStateMachine()`

- 作用：推进自动工位非阻塞状态机。
- 什么时候被调用：每次 `loop()` 中，在模式分发之前。
- 输入/输出：无参数，无返回值。
- 会影响哪些硬件：按阶段控制 M5、S1、S4。
- 调参相关：`AUTO_SERVO_SETTLE_MS`、`estimateMotor5MoveTime()`、各工位目标参数。
- 注意事项：它不使用长 `delay()`，而是依靠 `millis()` 判断当前阶段是否完成。

自动流程大致为：

```text
所有自动工位切换：
M5 -> HIGH_POS
S1/S4 -> 目标角度
等待 500ms
M5 -> 目标高度
完成
```

### `estimateMotor5MoveTime(int32_t from, int32_t to)`

- 作用：估算 M5 从当前位置移动到目标位置需要等待多久。
- 什么时候被调用：自动工位启动和阶段切换时。
- 输入/输出：输入起点和终点；返回等待毫秒数。
- 会影响哪些硬件：不直接控制硬件，但影响自动状态机等待时间。
- 调参相关：当前直接返回 `AUTO_M5_MOVE_WAIT_MS`。
- 注意事项：没有读取电机到位反馈时，这个值就是自动流程等待 M5 到位的依据。B2 反应慢就调小；S1/S4 旋转前 M5 还没回到安全高位就调大。

### `getAutoTargets(uint8_t state, int32_t& m5, int& s1, int& s4)`

- 作用：根据工位编号取出 M5、S1、S4 的目标值。
- 什么时候被调用：启动自动工位和推进自动状态机时。
- 输入/输出：输入状态 `1/2/3`；通过引用输出目标 M5/S1/S4。
- 会影响哪些硬件：不直接控制硬件，但决定自动工位最终动作。
- 调参相关：`PRE_CLAMP_MOTOR5`、`PRE_CLAMP_SERVO0`、`PRE_CLAMP_SERVO3`、`TURNTABLE_*`、`PRE_DROP_*`。
- 注意事项：S4 输出的是逻辑角度，真正写舵机时仍会自动执行 `180 - angle`。

### `transitionUsesHighPos(uint8_t fromState, uint8_t toState)`

- 作用：判断某次工位切换是否需要先经过安全高位 `HIGH_POS`。
- 什么时候被调用：`startAutoState()` 中。
- 输入/输出：输入当前工位和目标工位；返回是否需要先升到高位。
- 会影响哪些硬件：间接决定 M5 的运动路径。
- 调参相关：当前固定返回 `true`，也就是所有自动切换都会先回 `HIGH_POS`。
- 注意事项：这样可以保证旋转 S1/S4 前，M5 已经回到 1000 脉冲以内的安全高度，减少碰撞风险。

## 调试输出函数

### `printDebugInfoPeriodically()`

- 作用：周期打印遥控器通道、模式、failsafe、自动工位、电机和舵机状态。
- 什么时候被调用：`loop()` 多个分支末尾都会调用。
- 输入/输出：无参数，无返回值。
- 会影响哪些硬件：不影响硬件，只打印串口。
- 调参相关：`DEBUG_PRINT_MS`。
- 注意事项：如果串口刷屏太快，增大 `DEBUG_PRINT_MS`；如果想更实时，减小它。

输出格式类似：

```text
CH1=1500 CH2=1500 CH3=1500 CH4=1500 CH5=1000 CH6=1000 mode=DRIVE failsafe=0 auto=0 M1=0 M2=0 M3=0 M4=0 M5=0 M6=0 S1=180 S4=0 grip=90
```

## 调参速查表

| 现象 | 优先修改 |
| ---- | -------- |
| 遥控器通道对应不对 | `RC_CH1_INDEX` 到 `RC_CH6_INDEX` |
| 摇杆中位有小幅抖动 | 增大 `RC_DEADZONE` |
| 摇杆需要推很多才有反应 | 减小 `RC_DEADZONE` |
| 底盘前后方向反 | `CHASSIS_Y_SIGN` |
| 底盘左右横移方向反 | `CHASSIS_X_SIGN` |
| 原地转向方向反 | `CHASSIS_W_SIGN` |
| 单个轮子方向反 | `M1_SIGN` 到 `M4_SIGN` |
| 底盘太快 | 降低 `MAX_SPEED_RPM` 或 `WHEEL_CMD_MAX` |
| 底盘起步太敏感 | 增大 `MIN_EFFECTIVE_CMD` |
| M5 升降方向反 | `M5_DIR_SIGN` |
| M5 行程不对 | `M5_PULSE_MIN/MAX` |
| M5 每次动太多 | 减小 `M5_STEP` |
| M5 按住升降太快 | 减小 `M5_STEP` 或增大 `M5_UPDATE_MS` |
| M6 每步转动脉冲不合适 | 修改 `M6_STEP` |
| M6 回中脉冲位置不对 | 修改 `M6_CENTER` |
| 夹爪打不开或夹不紧 | `GRIPPER_OPEN_ANGLE` / `GRIPPER_CLOSE_ANGLE` |
| 夹爪两个舵机方向不匹配 | `GRIPPER_S3_REVERSED` |
| S4 方向看起来不对 | 先确认机械安装，再检查 `setServoLogical()` 的 S4 反向逻辑 |
| 自动工位高度不对 | `PRE_CLAMP_MOTOR5`、`TURNTABLE_MOTOR5`、`PRE_DROP_MOTOR5` |
| 自动工位 M5 太快 | 减小 `POSITION_SPEED_M5_AUTO` |
| 自动工位角度不对 | `PRE_CLAMP_SERVO0/SERVO3`、`TURNTABLE_SERVO0/SERVO3`、`PRE_DROP_SERVO0/SERVO3` |
| 自动工位等待太短 | 调整 `estimateMotor5MoveTime()` 或增大 `AUTO_SERVO_SETTLE_MS` |
| 关闭遥控器后急停太慢 | 减小 `RC_TIMEOUT_MS` |
| 遥控器偶发误触发 failsafe | 增大 `RC_TIMEOUT_MS`，检查接收机供电和 SBUS 线 |

## 实机调参建议顺序

1. 只接 ESP32 和接收机，确认 CH1~CH6 打印正确。
2. 只接 PCA9685 和舵机，调 S1/S2/S3/S4 的角度和方向。
3. 只接一个底盘电机，确认 RS485 地址和方向。
4. 架空底盘，调四轮方向宏和速度。
5. 单独测试 M5 升降，确认方向、行程和限位。
6. 单独测试 M6 物料盘，确认步长和回中。
7. 最后再测试自动工位，微调三个工位参数。

## 改代码时最容易踩的点

- `idx` 是 0 索引：`idx=0` 是 M1，`idx=4` 是 M5，`idx=5` 是 M6。
- `servoId` 是 1 索引：`servoId=1` 是 S1，`servoId=4` 是 S4。
- S4 的 `angle` 是逻辑角度，实际写入 PCA9685 前会变成 `180 - angle`。
- M5 的 `motor5Pos` 现在就是目标脉冲数，不再经过另一套位置单位换算。
- M6 的 `motor6Pos` 现在也是目标脉冲数，支持正负方向，方向由 `pos >= 0` 判断。
- `setMotorSpeed()` 有缓存，相同速度不会重复发命令。
- failsafe 恢复后必须所有摇杆回中，否则 `loop()` 不会进入正常控制。
- 自动工位运行时手动控制会暂停，防止 M5/S1/S4 被抢控制。
