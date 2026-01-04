<!-- -*- coding: utf-8 -*- -->
<!-- 本文件使用 UTF-8 编码，请在 VSCode 中确保使用 UTF-8 打开 -->
<!-- VSCode: 右下角点击编码 -> 选择 "通过编码重新打开" -> 选择 "UTF-8" -->

# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在此代码仓库中工作时提供指导。

**重要：在此项目中，请始终使用中文进行回答和交流。添加的代码注释为中英双语**

## 项目概述

这是一个基于 **STM32F4 的 RoboMaster 比赛机器人控制系统**，用于云台（gimbal）平台。固件控制底盘运动、云台定位（pitch/yaw）、射击机构，并与视觉系统接口实现自动瞄准。项目使用 FreeRTOS 进行任务管理，遵循 DJI RoboMaster 开发板标准。

**目标硬件**：STM32F4xx 微控制器（Type-C 开发板）
**操作系统**：FreeRTOS with CMSIS-RTOS 封装
**编译系统**：Keil MDK-ARM (μVision)
**通信方式**：CAN 总线用于电机控制，UART 用于遥控器（SBUS）和裁判系统

## 编译命令

本项目使用 Keil MDK-ARM。如果使用 VS Code 配合 Embedded IDE (EIDE) 扩展：

```bash
# 编译项目（从 MDK-ARM 目录或根目录）
# 在 VS Code 中：运行任务 "build" 或使用命令面板 -> "EIDE: Build"

# 清理编译
# 在 VS Code 中：运行任务 "clean"

# 重新编译（清理 + 编译）
# 在 VS Code 中：运行任务 "rebuild"

# 烧录到设备
# 在 VS Code 中：运行任务 "flash"

# 编译并烧录
# 在 VS Code 中：运行任务 "build and flash"
```

**注意**：原生 Keil 命令需要 Keil μVision IDE。`MDK-ARM/` 目录中的 `.vscode/tasks.json` 文件提供了基于 EIDE 的编译任务。

**项目文件**：
- `MDK-ARM/standard_tpye_c.uvprojx` - Keil 项目文件
- `standard_tpye_c.ioc` - STM32CubeMX 配置文件

## 代码架构

### 目录结构

```
├── application/          # 高层应用任务和控制逻辑
│   ├── *_task.c/h       # FreeRTOS 任务实现
│   ├── *_behaviour.c/h  # 行为/状态机逻辑
│   ├── CAN_receive.c/h  # 通过 CAN 接收电机反馈
│   ├── remote_control.c/h # 遥控器接收（SBUS 协议）
│   ├── referee.c/h      # 裁判系统协议
│   ├── shoot.c/h        # 射击控制
│   └── protocol/        # 通信协议
├── bsp/boards/          # 板级支持包 - 硬件抽象层
│   ├── bsp_can.c/h      # CAN 总线初始化
│   ├── bsp_rc.c/h       # 遥控器接收 UART+DMA
│   ├── bsp_usart.c/h    # 串口通信
│   └── bsp_*.c/h        # 其他外设驱动
├── components/          # 可复用软件组件
│   ├── algorithm/       # AHRS、DSP（ARM Math）、滤波器
│   ├── controller/      # PID 控制器
│   ├── devices/         # 设备驱动（BMI088 IMU、IST8310 磁力计、OLED）
│   └── support/         # 工具函数（CRC、FIFO、内存管理）
├── Drivers/             # STM32 HAL 和 CMSIS 库
├── Middlewares/         # FreeRTOS 和 USB 中间件
├── Src/                 # STM32CubeMX 生成的代码
│   ├── main.c           # 入口点，外设初始化
│   └── freertos.c       # FreeRTOS 任务创建
├── Inc/                 # 生成的头文件
└── MDK-ARM/             # Keil 项目和编译文件
    ├── AutoGimbal.c/h   # 视觉自瞄接口
    └── build/           # 编译生成的二进制文件
```

## 控制逻辑架构（application/ 详解）

### 双层控制架构

本项目采用 **行为层（Behaviour Layer）+ 任务层（Task Layer）** 的双层控制架构：

```
遥控器输入 → 行为状态机 → 任务控制器 → PID 控制 → 电机输出
```

#### 第一层：行为状态机（Behaviour State Machine）

**文件位置**：`application/gimbal_behaviour.c/h` 和 `application/chassis_behaviour.c/h`

**云台行为模式** (`gimbal_behaviour_e`)：
- `GIMBAL_ZERO_FORCE` - 零力矩模式，电机无输出
- `GIMBAL_INIT` - 初始化，云台归中
- `GIMBAL_CALI` - 校准模式
- `GIMBAL_ABSOLUTE_ANGLE` - 绝对角度控制（陀螺仪）
- `GIMBAL_RELATIVE_ANGLE` - 相对角度控制（编码器）
- `GIMBAL_MOTIONLESS` - 静止不动模式
- `GIMBAL_AUTO` - 视觉自瞄模式

**底盘行为模式** (`chassis_behaviour_e`)：
- `CHASSIS_ZERO_FORCE` - 零力矩，底盘无力
- `CHASSIS_NO_MOVE` - 底盘保持不动
- `CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW` - 步兵跟随云台模式（常用）
- `CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW` - 跟随底盘自身 yaw
- `CHASSIS_NO_FOLLOW_YAW` - 不跟随角度，小陀螺模式
- `CHASSIS_OPEN` - 开环模式，直接发送电流

**关键函数**：
- `gimbal_behaviour_mode_set()` - 根据遥控器输入设置云台行为状态
- `gimbal_behaviour_control_set()` - 根据当前行为调用对应控制函数
- `chassis_behaviour_mode_set()` - 设置底盘行为状态
- `chassis_behaviour_control_set()` - 底盘行为控制分发

#### 第二层：任务控制器（Task Controller）

**文件位置**：`application/gimbal_task.c/h` 和 `application/chassis_task.c/h`

**云台任务** (`gimbal_task`):
- 运行频率：1ms (1000Hz)
- 控制模式：
  - `GIMBAL_MOTOR_RAW` - 直接电流控制
  - `GIMBAL_MOTOR_GYRO` - 陀螺仪角度闭环
  - `GIMBAL_MOTOR_ENCONDE` - 编码器角度闭环
  - `GIMBAL_MOTOR_AUTO` - 视觉自瞄角度闭环
- 串级 PID：外环（角度） → 内环（速度）
- 每个轴（pitch/yaw）独立控制，支持不同 PID 参数

**底盘任务** (`chassis_task`):
- 运行频率：2ms (500Hz)
- 控制模式：
  - `CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW` - 速度控制 + 跟随云台角度
  - `CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW` - 速度控制 + 跟随底盘绝对角度
  - `CHASSIS_VECTOR_NO_FOLLOW_YAW` - 速度控制 + 旋转速度控制
  - `CHASSIS_VECTOR_RAW` - 原始电流模式
- 功率限制：通过 `chassis_power_control.c` 限制电流防止超功率

### 射击系统

**文件位置**：`application/shoot.c/h`

**射击状态机** (`shoot_mode_e`)：
```
SHOOT_STOP → SHOOT_READY_FRIC → SHOOT_READY_BULLET → SHOOT_READY →
SHOOT_BULLET / SHOOT_CONTINUE_BULLET → SHOOT_DONE
```

- `SHOOT_STOP` - 停止射击
- `SHOOT_READY_FRIC` - 摩擦轮准备（加速）
- `SHOOT_READY_BULLET` - 弹仓准备
- `SHOOT_READY` - 就绪
- `SHOOT_BULLET` - 单发射击
- `SHOOT_CONTINUE_BULLET` - 连发射击
- `SHOOT_DONE` - 射击完成

**关键组件**：
- **摩擦轮电机**：左右两个电机（fricL / fricR），使用速度闭环 PID
- **拨弹电机**：2006 电机，角度/速度控制
- **热量管理**：从裁判系统读取热量数据，防止超热量

**控制流程**：
1. 检测 `shoot_flag`（来自底盘通信数据）
2. 根据 flag 设置摩擦轮目标转速
3. PID 控制摩擦轮达到目标弹速
4. 控制拨弹盘发射

### 通信系统

#### 1. 遥控器通信 (`remote_control.c/h`)
- **协议**：SBUS（Futaba 协议）
- **接口**：USART3，波特率 100000
- **传输方式**：DMA + 空闲中断
- **数据结构**：`RC_ctrl_t` 包含：
  - 4 通道摇杆值（ch[0-3]）
  - 2 个三档开关（s[0-1]）
  - 鼠标数据（x, y, z, 左右键）
  - 键盘数据（16 位按键标志）

#### 2. CAN 总线通信 (`CAN_receive.c/h`)
- **CAN1**：底盘电机（0x201-0x204）、超级电容（0x211）
- **CAN2**：云台电机（Yaw: 0x09, Pitch: 0x05）、拨弹/摩擦轮电机
- **中断接收**：`HAL_CAN_RxFifo0MsgPendingCallback()` 解析电机反馈
- **电机数据**：编码器位置、转速、电流、温度
- **发送函数**：
  - `CAN_cmd_gimbal()` - 发送云台电机电流
  - `CAN_cmd_chassis()` - 发送底盘电机电流
  - `CAN_cmd_fric()` - 发送摩擦轮电机电流

#### 3. 视觉自瞄通信 (`MDK-ARM/AutoGimbal.c/h`)
- **接口**：UART 与上位机（视觉处理器）通信
- **数据结构**：`CTRL` 包含 x/y 像素偏移、距离、模式
- **发送数据**：`AUTO_SEND_TO_NUC_DATA_t` 发送当前姿态（roll/pitch/yaw）给视觉
- **灵敏度参数**：
  - `PITCH_AUTO_SEN` - pitch 轴自瞄灵敏度
  - `YAW_AUTO_SEN` - yaw 轴自瞄灵敏度

#### 4. 裁判系统通信 (`referee.c/h`)
- **协议**：RoboMaster 官方裁判系统协议
- **数据包括**：
  - 比赛状态（`game_status_t`）
  - 机器人状态（`robot_status_t`）- HP、等级、位置
  - 功率热量数据（`power_heat_data_t`）- 底盘功率、枪口热量
  - 伤害数据
  - 射击数据
  - 机器人间通信

### 任务架构（FreeRTOS）

系统运行多个并发任务，定义在 `Src/freertos.c` 中：

1. **`gimbal_task`** - 云台控制，1ms 周期
2. **`chassis_task`** - 底盘控制，2ms 周期
3. **`shoot_control_loop`** - 射击控制（在 gimbal_task 中调用）
4. **`INS_task`** - 惯导姿态解算，提供 roll/pitch/yaw
5. **`detect_task`** - 设备在线检测，监控电机/传感器连接状态
6. **`calibrate_task`** - IMU 和云台校准
7. **`referee_usart_task`** - 裁判系统数据接收
8. **`led_flow_task`** - RGB LED 流水灯状态指示
9. **`oled_task`** - OLED 显示
10. **`voltage_task`** - 电池电压监测
11. **`usb_task`** - USB 虚拟串口
12. **`servo_task`** - 舵机控制（可选）

### 关键数据流

```
┌─────────────┐
│  遥控器输入  │ (SBUS via UART3 DMA)
└─────────────┘
       │
       ↓
┌───────────────────────┐
│  remote_control.c   │ → RC_ctrl_t 结构体
└───────────────────────┘
       │
       ↓
┌─────────────────────────────────────────┐
│  行为层（Behaviour State Machine）     │
│  - gimbal_behaviour_mode_set()       │
│  - chassis_behaviour_mode_set()      │
└─────────────────────────────────────────┘
       │
       ↓
┌─────────────────────────────────────────┐
│  行为控制分发                          │
│  - gimbal_behaviour_control_set()    │
│  - chassis_behaviour_control_set()   │
└─────────────────────────────────────────┘
       │
       ↓
┌─────────────────────────────────────────┐
│  任务层（Task Controllers）            │
│  - gimbal_task.c                     │
│  - chassis_task.c                    │
│  - shoot.c                           │
└─────────────────────────────────────────┘
       │
       ↓
┌─────────────────────────────────────────┐
│  PID 控制器                            │
│  - 云台：角度环 + 速度环                 │
│  - 底盘：速度环 + 跟随角度环             │
│  - 射击：摩擦轮速度环                    │
└─────────────────────────────────────────┘
       │
       ↓
┌─────────────────────────────────────────┐
│  CAN 总线输出                          │
│  - CAN_cmd_gimbal()                  │
│  - CAN_cmd_chassis()                 │
│  - CAN_cmd_fric()                    │
└─────────────────────────────────────────┘
       │
       ↓
┌─────────────────────────────────────────┐
│  电机执行                              │
└─────────────────────────────────────────┘

【反馈回路】
电机反馈（编码器、速度、电流）→ CAN 中断接收 →
motor_measure_t 结构体 → PID 控制器
```

### PID 调参

PID 参数在任务头文件中定义为宏常量：

**云台 PID** (`gimbal_task.h`):
- Pitch 速度环：`PITCH_SPEED_PID_KP/KI/KD`
- Yaw 速度环：`YAW_SPEED_PID_KP/KI/KD`
- Yaw 陀螺仪角度环：`YAW_GYRO_ABSOLUTE_PID_KP/KI/KD`
- Pitch 陀螺仪角度环：`PITCH_GYRO_ABSOLUTE_PID_KP/KI/KD`
- **自瞄专用 PID**：
  - `YAW_AUTO_ABSOLUTE_PID_KP/KI/KD` - 自瞄 yaw 角度环
  - `PITCH_AUTO_ABSOLUTE_PID_KP/KI/KD` - 自瞄 pitch 角度环
  - `YAW_AUTO_LOW_ABSOLUTE_PID_*` - 接近目标时的低速 PID

**底盘 PID** (`chassis_task.h`):
- 电机速度环：`M3505_MOTOR_SPEED_PID_KP/KI/KD`
- 跟随云台角度环：`CHASSIS_FOLLOW_GIMBAL_PID_KP/KI/KD`
- 功率限制 PID：`POWER_PID_KP/KI/KD`

**射击 PID** (`shoot.h`):
- 摩擦轮速度环：`FRIC_SPEED_PID_KP/KI/KD`
- 拨弹角度环：`TRIGGER_ANGLE_PID_KP/KI/KD`

### CAN 总线配置

- **CAN1**（底盘总线）：
  - 0x201-0x204：M3508 底盘电机 1-4
  - 0x211：超级电容
  - 0x10：底盘间通信 ID

- **CAN2**（云台总线）：
  - 0x09：Yaw 云台电机（可能是 6020 或 GM6020）
  - 0x05：Pitch 云台电机
  - 0x207：拨弹电机（2006）
  - 摩擦轮电机（具体 ID 见 `CAN_receive.h`）

- **发送 ID**：
  - 云台：`CAN_GIMBAL_ALL_ID = 0x1FF`
  - 底盘：`CAN_CHASSIS_ALL_ID = 0x200`

## 如何添加新的控制模式

### 添加云台新行为

1. 在 `gimbal_behaviour.h` 的 `gimbal_behaviour_e` 枚举中添加新模式
2. 实现控制函数：`gimbal_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)`
3. 在 `gimbal_behaviour_mode_set()` 中添加模式切换逻辑
4. 在 `gimbal_behaviour_control_set()` 末尾添加模式分支

### 添加底盘新行为

1. 在 `chassis_behaviour.h` 的 `chassis_behaviour_e` 枚举中添加新模式
2. 实现控制函数：`chassis_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t *chassis)`
3. 在 `chassis_behaviour_mode_set()` 中添加切换逻辑
4. 在 `chassis_behaviour_control_set()` 中添加分发

## 硬件抽象

**对于硬件相关的修改，始终修改 BSP 文件**（`bsp/boards/bsp_*.c`），而不是 `Src/` 中 HAL 生成的代码。

**重要**：`Src/` 和 `Inc/` 由 STM32CubeMX 自动生成。重新生成时，保留 `/* USER CODE BEGIN */` 和 `/* USER CODE END */` 块之间的代码。

## 常见问题

- **电机无响应**：检查 `CAN_receive.h` 中的 CAN ID 映射和接线
- **云台漂移**：验证 IMU 校准（运行 calibrate_task），检查 AHRS 数据质量
- **遥控器不工作**：SBUS 信号在 USART3 上，验证波特率（100000）和空闲线检测
- **CubeMX 重新生成后编译错误**：检查用户代码块是否被覆盖
- **自瞄抖动**：检查 `AutoGimbal.h` 中的灵敏度参数，调整自瞄 PID
- **底盘超功率**：检查 `chassis_power_control.c` 中的功率限制逻辑和裁判系统数据
- **射击卡弹**：检查拨弹电机堵转检测和反转逻辑

## 调试技巧

1. **使用 OLED 显示调试信息**：在 `oled_task.c` 中添加需要显示的变量
2. **LED 状态指示**：`led_flow_task.c` 通过不同颜色指示系统状态
3. **detect_task 监控**：查看哪些设备离线，定位通信问题
4. **USB 虚拟串口**：通过 `usb_task.c` 输出调试信息到电脑
