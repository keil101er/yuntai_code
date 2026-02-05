<!-- -*- coding: utf-8 -*- -->
<!-- 本文件使用 UTF-8 编码，请在 VSCode 中确保使用 UTF-8 打开 -->
<!-- VSCode: 右下角点击编码 -> 选择 "通过编码重新打开" -> 选择 "UTF-8" -->

# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在此代码仓库中工作时提供指导，当代码与本文件描述产生冲突时，以实际代码为准。

**重要：在此项目中，请始终使用中文进行回答和交流。添加的代码注释为中英双语**

**重要说明**：此项目是在一个框架上编写而成的，有许多函数和代码并没有参与机器人的实际控制，所以读取到相关代码时，不要认为它就被实际使用了。现在实际运行的FreeRTOS任务有test_task,calibrate_task,gimbal_task,INS_task,led_RGB_flow_task,referee_usart_task,usb_task,battery_voltage_task,user_task，当你不确定一个函数或什么东西是否真正有用时可以从这些任务函数中寻找。

---

## 目录

1. [项目概述](#项目概述)
2. [快速开始](#快速开始)
3. [代码架构](#代码架构)
4. [控制逻辑详解](#控制逻辑详解)
5. [双板系统架构](#双板系统架构)
6. [PID调参参考](#pid调参参考)
7. [调试与问题排查](#调试与问题排查)
8. [附录](#附录)

---

## 项目概述

这是一个基于 **STM32F4 的 RoboMaster 比赛机器人控制系统**，用于云台（gimbal）平台。固件控制云台定位（pitch/yaw）、射击机构，并与视觉系统接口实现自动瞄准。项目使用 FreeRTOS 进行任务管理，遵循 DJI RoboMaster 开发板标准。

### 系统定位

**本项目为云台板（上板）控制代码**，与底盘板（下板）协同工作：

- **云台板职责**：云台姿态稳定、射击控制、视觉自瞄、遥控器解析、裁判系统通信
- **底盘板职责**：轮腿运动控制（VMC）、平衡控制（LQR）、底盘跟随云台、功率管理
- **通信方式**：底盘板通过 **CAN1 共享总线** 读取云台 Yaw 电机编码器实现跟随（Yaw 电机挂载在 CAN1 上，两板都能接收数据；⚠️ 备用 CAN ID 0x15 协议已实现但未启用）

### 硬件配置

- **目标硬件**：STM32F4xx 微控制器（Type-C 开发板）
- **操作系统**：FreeRTOS with CMSIS-RTOS 封装
- **编译系统**：Keil MDK-ARM (μVision)
- **主要外设**：
  - **CAN1**（共享总线，与底盘板互通）：Yaw 云台电机（0x09）、拨弹电机（0x207）、板间通信（0x10/0x15）
  - **CAN2**（私有总线）：Pitch 云台电机（0x05）、摩擦轮电机（0x205/0x206）
  - **UART3**：遥控器（SBUS，100000 波特率）
  - **UART6**：裁判系统
  - **UART1**：视觉上位机

---

## 快速开始

### 项目文件

- `MDK-ARM/standard_tpye_c.uvprojx` - Keil 项目文件
- `standard_tpye_c.ioc` - STM32CubeMX 配置文件

### 编译方式

#### 方式1：VS Code + EIDE 扩展（推荐）

```bash
# 编译项目
运行任务 "build" 或命令面板 -> "EIDE: Build"

# 清理编译
运行任务 "clean"

# 烧录到设备
运行任务 "flash"
```

#### 方式2：Keil 命令行编译

**前提条件**：
- 已安装 Keil MDK-ARM v5（通常位于 `C:\Keil_v5\`）
- 在命令行中进入 `MDK-ARM/` 目录

**编译命令**：
```bash
# 标准编译
"C:\Keil_v5\UV4\UV4.exe" -b standard_tpye_c.uvprojx -o build_log.txt

# 查看编译结果
type build_log.txt
```

**成功标志**：
```
"standard_tpye_c\standard_tpye_c.axf" - 0 Error(s), 0 Warning(s).
```

**输出文件**：
- `standard_tpye_c.axf` - ELF 可执行文件
- `standard_tpye_c.hex` - HEX 固件（用于烧录）
- `standard_tpye_c.bin` - BIN 固件

### 编译输出分析

```
Program Size: Code=XXXXX RO-data=XXXX RW-data=XXXX ZI-data=XXXXX
```

- **Flash 使用量** = Code + RO-data + RW-data
- **RAM 使用量** = RW-data + ZI-data

**性能基准**（STM32F407VET6）：
- Flash 限制：512KB
- RAM 限制：128KB
- 典型使用率：Flash ~15%，RAM ~44%

### 常见编译错误

| 错误类型 | 示例 | 解决方法 |
|---------|------|---------|
| 函数名拼写错误 | `error: #70: incomplete type` | 检查函数声明和定义 |
| 头文件缺失 | `error: #5: cannot open source input file` | 检查 Keil Include Paths 设置 |
| 未定义引用 | `error: L6218E: Undefined symbol` | 检查 .c 文件是否已添加到项目 |
| 内存溢出 | `error: L6407E: Sections could not fit` | 优化代码或使用更大 Flash 芯片 |

---

## 代码架构

### 目录结构

```
├── application/          # 高层应用任务和控制逻辑
│   ├── *_task.c/h       # FreeRTOS 任务实现
│   ├── *_behaviour.c/h  # 行为/状态机逻辑
│   ├── CAN_receive.c/h  # CAN 接收电机反馈
│   ├── remote_control.c/h # 遥控器接收（SBUS）
│   ├── referee.c/h      # 裁判系统协议
│   └── shoot.c/h        # 射击控制
├── bsp/boards/          # 板级支持包 - 硬件抽象层
│   ├── bsp_can.c/h      # CAN 总线初始化
│   ├── bsp_rc.c/h       # 遥控器 UART+DMA
│   └── bsp_usart.c/h    # 串口通信
├── components/          # 可复用软件组件
│   ├── algorithm/       # AHRS、滤波器
│   ├── controller/      # PID 控制器
│   └── devices/         # 设备驱动（IMU、磁力计、OLED）
├── Drivers/             # STM32 HAL 和 CMSIS 库
├── Middlewares/         # FreeRTOS 和 USB 中间件
├── Src/                 # STM32CubeMX 生成的代码
│   ├── main.c           # 入口点
│   └── freertos.c       # FreeRTOS 任务创建
├── Inc/                 # 生成的头文件
└── MDK-ARM/             # Keil 项目和编译文件
    └── AutoGimbal.c/h   # 视觉自瞄接口
```

### FreeRTOS 任务架构

系统运行多个并发任务，定义在 `Src/freertos.c` 中：

| 任务名 | 功能 | 周期 |
|--------|------|------|
| `gimbal_task` | 云台控制 | 1ms (1000Hz) |
| `shoot_control_loop` | 射击控制 | 在 gimbal_task 中调用 |
| `INS_task` | 惯导姿态解算 | 1ms |
| `detect_task` | 设备在线检测 | - |
| `calibrate_task` | IMU 和云台校准 | - |
| `referee_usart_task` | 裁判系统数据接收 | - |
| `led_flow_task` | RGB LED 状态指示 | - |
| `oled_task` | OLED 显示 | - |
| `voltage_task` | 电池电压监测 | - |
| `usb_task` | USB 虚拟串口 | - |

### 硬件抽象层注意事项

**重要**：`Src/` 和 `Inc/` 由 STM32CubeMX 自动生成。重新生成时，保留 `/* USER CODE BEGIN */` 和 `/* USER CODE END */` 块之间的代码。

**对于硬件相关的修改，始终修改 BSP 文件**（`bsp/boards/bsp_*.c`），而不是 `Src/` 中 HAL 生成的代码。

---

## 控制逻辑详解

### 双层控制架构

本项目采用 **行为层（Behaviour Layer）+ 任务层（Task Layer）** 的双层控制架构：

```
遥控器输入 → 行为状态机 → 任务控制器 → PID 控制 → 电机输出
```

#### 第一层：行为状态机

**文件位置**：`application/gimbal_behaviour.c/h`

**云台行为模式** (`gimbal_behaviour_e`)：
- `GIMBAL_ZERO_FORCE` - 零力矩模式
- `GIMBAL_INIT` - 初始化归中
- `GIMBAL_CALI` - 校准模式
- `GIMBAL_ABSOLUTE_ANGLE` - 绝对角度控制（陀螺仪）
- `GIMBAL_RELATIVE_ANGLE` - 相对角度控制（编码器）
- `GIMBAL_MOTIONLESS` - 静止不动
- `GIMBAL_AUTO` - 视觉自瞄

**关键函数**：
- `gimbal_behaviour_mode_set()` - 根据遥控器输入设置行为状态
- `gimbal_behaviour_control_set()` - 根据当前行为调用对应控制函数

#### 第二层：任务控制器

**文件位置**：`application/gimbal_task.c/h`

**云台任务控制模式**：
- `GIMBAL_MOTOR_RAW` - 直接电流控制
- `GIMBAL_MOTOR_GYRO` - 陀螺仪角度闭环
- `GIMBAL_MOTOR_ENCONDE` - 编码器角度闭环
- `GIMBAL_MOTOR_AUTO` - 视觉自瞄角度闭环

**控制特点**：
- 运行频率：1ms (1000Hz)
- 串级 PID：外环（角度） → 内环（速度）
- 每个轴（pitch/yaw）独立控制

### 射击系统

**文件位置**：`application/shoot.c/h`

**射击状态机**：
```
SHOOT_STOP → SHOOT_READY_FRIC → SHOOT_READY_BULLET → SHOOT_READY →
SHOOT_BULLET / SHOOT_CONTINUE_BULLET → SHOOT_DONE
```

**关键组件**：
- **摩擦轮电机**：左右两个电机（fricL/fricR），速度闭环 PID
- **拨弹电机**：2006 电机，角度/速度控制
- **热量管理**：从裁判系统读取热量数据，防止超热量

### 通信系统

#### 1. 遥控器通信 (`remote_control.c/h`)
- **协议**：SBUS（Futaba 协议）
- **接口**：USART3，波特率 100000
- **传输方式**：DMA + 空闲中断
- **数据结构**：`RC_ctrl_t` 包含摇杆、开关、鼠标、键盘数据

#### 2. CAN 总线通信 (`CAN_receive.c/h`)

**⚠️ 重要：CAN 总线拓扑结构**

系统采用 **共享 CAN1 + 私有 CAN2** 的双总线架构：

```
        云台板CAN1 ←→ 中心板 ←→ 底盘板CAN1  （共享总线，两板互通）
             ↓                        ↓
        云台板CAN2                底盘板CAN2  （各板私有，互不干扰）
```

**CAN1（共享总线）** - 云台板和底盘板通过中心板连接，两板都能接收此总线上的数据：
- 0x09：Yaw 云台电机（DM4310）- **底盘板通过此总线读取 Yaw 编码器实现跟随**
- 0x207：拨弹电机（2006）
- 0x10：板间通信接收 ID（底盘板发送给云台板）
- 0x15：板间通信发送 ID（云台板发送给底盘板，⚠️ 未启用）
- 0x201-0x204：M3508 底盘电机（主要由底盘板使用）

**CAN2（云台板私有总线）** - 仅云台板本地使用：
- 0x05：Pitch 云台电机（6020 或 DM4310）
- 0x205：摩擦轮右电机（M3508）
- 0x206：摩擦轮左电机（M3508）

**关键函数**：
- `CAN_cmd_gimbal()` - 发送云台电机电流
- `CAN_cmd_chassis()` - 发送底盘电机电流
- `CAN_cmd_fric()` - 发送摩擦轮电机电流

#### 3. 视觉自瞄通信 (`MDK-ARM/AutoGimbal.c/h`)
- **接口**：UART 与视觉上位机通信
- **数据结构**：`CTRL` 包含 x/y 像素偏移、距离、模式
- **灵敏度参数**：
  - `PITCH_AUTO_SEN` - pitch 轴自瞄灵敏度
  - `YAW_AUTO_SEN` - yaw 轴自瞄灵敏度

#### 4. 裁判系统通信 (`referee.c/h`)
- **协议**：RoboMaster 官方裁判系统协议
- **数据包括**：比赛状态、机器人状态、功率热量数据、伤害数据、射击数据

### 关键数据流

```
遥控器(SBUS) → remote_control.c → RC_ctrl_t
                      ↓
            gimbal_behaviour_mode_set()
                      ↓
         gimbal_behaviour_control_set()
                      ↓
                gimbal_task.c
                      ↓
                  PID 控制器
                      ↓
            CAN_cmd_gimbal() → 电机

【反馈回路】
电机反馈（编码器、速度） → CAN 中断 → motor_measure_t → PID
```

### 如何添加新的控制模式

1. 在 `gimbal_behaviour.h` 的 `gimbal_behaviour_e` 枚举中添加新模式
2. 实现控制函数：`gimbal_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)`
3. 在 `gimbal_behaviour_mode_set()` 中添加模式切换逻辑
4. 在 `gimbal_behaviour_control_set()` 末尾添加模式分支

---

## 双板系统架构

### 机器人整体架构

本 RoboMaster 机器人采用 **双板分离式架构**，由云台板（本项目）和底盘板两个独立控制器协同工作：

```
┌─────────────────────────────────────────────────────────────────┐
│                    RoboMaster 机器人系统                         │
├──────────────────────────┬──────────────────────────────────────┤
│   云台板（本项目）        │         底盘板                        │
│   STM32F4xx              │         STM32F407IGHx                 │
├──────────────────────────┼──────────────────────────────────────┤
│ 【硬件控制】             │ 【硬件控制】                          │
│ • Yaw 轴电机 (DM4310)    │ • 4个关节电机 DM8009（腿部）          │
│ • Pitch 轴电机 (6020)    │ • 4个轮毂电机 M3508（驱动轮）         │
│ • 摩擦轮电机 (左右各1个) │ • 底盘 IMU (BMI088 + IST8310)        │
│ • 拨弹电机 (2006)        │                                      │
│ • 云台 IMU (BMI088)      │                                      │
├──────────────────────────┼──────────────────────────────────────┤
│ 【控制任务】             │ 【控制任务】                          │
│ • 云台姿态稳定           │ • VMC 虚拟模型约束腿部控制            │
│ • 射击控制（热量管理）   │ • 底盘平衡控制（LQR）                 │
│ • 视觉自瞄               │ • 底盘跟随云台                        │
│ • 遥控器接收与解析       │ • 功率管理                            │
│ • 裁判系统通信           │ • 姿态估计与观测                      │
├──────────────────────────┼──────────────────────────────────────┤
│ 【对外接口】             │ 【对外接口】                          │
│ • UART3: 遥控器(SBUS)    │ • CAN1: 共享总线（与云台板互通）      │
│ • UART6: 裁判系统        │ • CAN2: 私有总线（腿部电机）          │
│ • UART1: 视觉上位机      │ • UART6: 调试串口                     │
│ • CAN1: 共享总线(Yaw等)  │                                      │
│ • CAN2: 私有总线(Pitch等)│                                      │
└──────────────────────────┴──────────────────────────────────────┘
```

### 云台板与底盘板通信机制

#### ⭐ 实际使用的跟随机制（通过共享 CAN1 总线）

**当前系统实际采用的底盘跟随云台方式**：底盘板直接通过 **共享 CAN1 总线** 读取云台 Yaw 电机（DM4310，ID=0x09）的编码器值，而不是通过专用的板间通信协议（0x15）。

**关键原理**：云台 Yaw 电机挂载在 CAN1 共享总线上，云台板和底盘板都能接收到该电机的反馈数据。

**数据流路径**：

1. **云台 Yaw 电机反馈** → CAN1 共享总线（云台板和底盘板都能接收）

2. **底盘板接收电机数据**：
   - 文件：底盘板 `application/CAN_receive.c:109-127`
   - 接收云台 Yaw 电机（ID=9）的编码器、速度、电流等反馈数据
   - 存储到：`chassis_move_balance.motor_chassis[4].ecd`

3. **底盘板计算云台相对角度**：
   - 文件：底盘板 `MDK-ARM/chassisL_task.c:176`
   ```c
   chassis->yaw_motor_angle = motor_ecd_to_angle_change(chassis->motor_chassis[4].ecd, 0);
   ```
   - 将编码器值转换为相对角度（注意：offset=0，即以编码器零点为参考）

4. **底盘板使用云台角度进行跟随控制**：
   - 文件：底盘板 `application/chassisR_task.c:653`
   ```c
   chassis->relative_angle = chassis->yaw_motor_angle;
   ```

**关键特点**：
- ✅ **实时性高**：直接读取电机反馈，无额外通信延迟
- ✅ **可靠性强**：利用 CAN1 共享总线特性，无需额外通信协议
- ✅ **硬件简单**：云台 Yaw 电机本就在 CAN1 上，底盘板自动接收数据
- ⚠️ **依赖编码器零点**：底盘跟随的准确性依赖于云台电机 `offset_ecd` 的正确设置
- ⚠️ **无云台姿态信息**：底盘无法获知云台的 IMU 绝对姿态，仅知道电机机械位置


#### 备用通信协议（CAN ID 0x15，当前未启用）

**⚠️ 注意**：以下协议代码已实现但**当前被注释掉**（`gimbal_task.c:286`），实际系统未使用此通信方式。

**设计目的**：发送云台板 IMU 的绝对 Yaw 角度到底盘板，用于更高级的姿态融合控制。

**函数**：`CAN_gimbal_send__to_chassis(CAN_HandleTypeDef *hcan, float UP_INS_YAW)`
**文件位置**：`application/CAN_receive.c:613-626`
**调用位置**：`application/gimbal_task.c:286`（被注释）
**通信 ID**：0x15（CAN1 总线）

**数据帧格式**：
```c
// 8字节数据帧
Byte 0-1: Yaw 角度（int16_t，压缩）
Byte 2-7: 保留

// 压缩算法：float (-3.5~3.5 rad) → int16 (-32767~32767)
int16_t YAW_NAW = (int16_t)((UP_INS_YAW / 3.5f) * 32767);
```

**启用方法**：
1. 云台板：取消注释 `gimbal_task.c:286`
2. 底盘板：修改控制逻辑，使用 `C_data.received_upyaw` 替代 `yaw_motor_angle`
3. 底盘板：添加通信超时检测

**优缺点对比**：
- ✅ 优点：可获取云台 IMU 绝对姿态
- ❌ 缺点：增加 CAN 总线负载
- ❌ 缺点：需要修改底盘板控制逻辑

---

## PID调参参考

PID 参数在任务头文件中定义为宏常量：

### 云台 PID (`gimbal_task.h`)

**速度环**：
- Pitch 速度环：`PITCH_SPEED_PID_KP/KI/KD`
- Yaw 速度环：`YAW_SPEED_PID_KP/KI/KD`

**角度环（陀螺仪）**：
- Yaw 陀螺仪角度环：`YAW_GYRO_ABSOLUTE_PID_KP/KI/KD`
- Pitch 陀螺仪角度环：`PITCH_GYRO_ABSOLUTE_PID_KP/KI/KD`

**自瞄专用 PID**：
- Yaw 自瞄角度环：`YAW_AUTO_ABSOLUTE_PID_KP/KI/KD`
- Pitch 自瞄角度环：`PITCH_AUTO_ABSOLUTE_PID_KP/KI/KD`
- Yaw 接近目标低速 PID：`YAW_AUTO_LOW_ABSOLUTE_PID_*`

### 底盘 PID (`chassis_task.h`)

- 电机速度环：`M3505_MOTOR_SPEED_PID_KP/KI/KD`
- 跟随云台角度环：`CHASSIS_FOLLOW_GIMBAL_PID_KP/KI/KD`
- 功率限制 PID：`POWER_PID_KP/KI/KD`

### 射击 PID (`shoot.h`)

- 摩擦轮速度环：`FRIC_SPEED_PID_KP/KI/KD`
- 拨弹角度环：`TRIGGER_ANGLE_PID_KP/KI/KD`

---

## 调试与问题排查

### 云台板调试方法

#### 1. 使用 OLED 显示调试信息
在 `oled_task.c` 中添加需要显示的变量

#### 2. LED 状态指示
`led_flow_task.c` 通过不同颜色指示系统状态

#### 3. detect_task 监控
查看哪些设备离线，定位通信问题

#### 4. USB 虚拟串口调试
```c
#include "usb_task.h"
char debug_buf[50];
sprintf(debug_buf, "Yaw: %.2f\r\n", gimbal_yaw_angle);
usb_printf(debug_buf);
```

#### 5. 性能分析（DWT）
```c
#include "bsp_dwt.h"

uint32_t start_time = DWT_GetTimeline_us();
// 执行控制代码...
uint32_t exec_time = DWT_GetTimeline_us() - start_time;
if (exec_time > 1000) {  // 超过 1ms 报警
    usb_printf("WARNING: Task exec time: %d us\r\n", exec_time);
}
```

### 常见问题排查

#### 云台板问题

| 问题 | 检查项 |
|------|--------|
| 电机无响应 | 检查 `CAN_receive.h` 中的 CAN ID 映射和接线 |
| 云台漂移 | 验证 IMU 校准（运行 calibrate_task），检查 AHRS 数据质量 |
| 遥控器不工作 | SBUS 信号在 USART3 上，验证波特率（100000）和空闲线检测 |
| CubeMX 重新生成后编译错误 | 检查用户代码块是否被覆盖 |
| 自瞄抖动 | 检查 `AutoGimbal.h` 中的灵敏度参数，调整自瞄 PID |
| 射击卡弹 | 检查拨弹电机堵转检测和反转逻辑 |

#### 底盘跟随云台问题（基于电机编码器）

**⚠️ 注意**：当前系统底盘通过直接读取云台 Yaw 电机编码器实现跟随，而非通过 CAN ID 0x15 通信。

**底盘不跟随云台或朝向错误**：
1. 检查云台 Yaw 电机是否正常工作（CAN ID=9）
2. 验证底盘板能否正确接收云台电机反馈数据
3. 检查云台电机 `offset_ecd` 设置是否正确（云台板 `gimbal_task.c:405`）
4. **如果云台反装**：需要修改 `offset_ecd`，加上 180° 偏移（4096）
5. 确认底盘板处于跟随模式（`CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW`）

**底盘跟随方向反了**：
1. 检查云台电机编码器零点设置（`offset_ecd`）
2. 可能是云台反装但未修正编码器零点
3. 解决方法：在 `set_cali_gimbal_hook()` 中调整 `offset_ecd`

**底盘跟随抖动**：
1. 检查云台电机编码器数据稳定性
2. 在底盘板侧添加低通滤波器平滑编码器数据
3. 调整底盘板跟随 PID 参数（减小 Kp，增加 Kd）
4. 检查云台 Yaw 轴机械安装是否松动

**调试建议**：
- 使用 CAN 分析仪监控云台 Yaw 电机（ID=9）的反馈数据
- 在底盘板代码中输出 `chassis->yaw_motor_angle`，观察编码器角度变化
- 在云台板代码中输出 `gimbal_control.gimbal_yaw_motor.offset_ecd`，验证零点设置

#### 整体系统问题

**底盘超功率**：
- 检查底盘板 `chassis_power_control.c` 中的功率限制逻辑和裁判系统数据

**机器人失控**：
1. 检查遥控器连接状态（云台板 `detect_task`��
2. 验证两块板的 IMU 校准状态
3. 确认安全保护机制是否触发（如通信超时保护）

### 双板系统调试建议

**分板调试流程**：
1. 先单独调试云台板：确保云台稳定、射击正常、IMU 数据准确
2. 再单独调试底盘板：确保站立平衡、腿部运动学正确
3. 最后联调：测试底盘跟随功能

**通信调试**（如果启用 CAN ID 0x15 协议）：
- 使用 CAN 分析仪监控 0x15 数据帧
- 在底盘板串口输出 `C_data.received_upyaw`
- 检查通信延迟

**性能优化**：
- 底盘跟随抖动：检查 PID 参数（底盘板 `TURN_PID_KP/KD`）
- 跟随响应慢：提高通信频率（取消注释 `gimbal_task.c:286`）
- CAN 总线负载高：降低板间通信频率或减少数据量

---

## 附录

### 底盘板项目信息

**项目路径**：`D:\RM_code\code_rmuc\chassis\1.12VMC\1.12\`

**核心特性**：
- **芯片**：STM32F407IGHx（192KB RAM）
- **控制算法**：VMC（虚拟模型约束）+ LQR 平衡控制
- **任务频率**：2ms（500Hz）腿部控制，1ms（1000Hz）IMU 融合
- **特殊电机**：DM8009 关节电机（位置/速度/力矩多模式）

**底盘板行为模式**：
- `CHASSIS_ZERO_FORCE`：零力矩
- `CHASSIS_BALANCE`：平衡模式
- `CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW`：跟随云台（常用）
- `CHASSIS_NO_FOLLOW_YAW`：小陀螺模式

**关键文件**：
- `application/chassisR_task.c/h`：右腿控制
- `application/chassisL_task.c/h`：左腿控制
- `components/algorithm/VMC_calc.c/h`：VMC 运动学
- `bsp/boards/CANdata_analysis.c/h`：接收云台板数据解析

### 备用板间通信协议代码参考（当前未使用）

**⚠️ 重要提示**：以下代码是关于 CAN ID 0x15 通信协议的，该协议当前被注释掉（未启用）。实际系统使用直接读取云台电机编码器的方式。

#### 步骤 1：CAN 中断接收（底盘板）

```c
// 文件：底盘板 application/CAN_receive.c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if(hcan == &hcan1) {
        switch (rx_header.StdId) {
            case 0x15:  // 云台板数据
                C_fbdata1(&C_data, rx_data, 8);
                break;
        }
    }
}
```

#### 步骤 2：数据解析（底盘板）

```c
// 文件：底盘板 bsp/boards/CANdata_analysis.c
void C_fbdata1(c_fbpara_t *motor, uint8_t *rx_data, uint32_t data_len)
{
    if(data_len == 8) {
        // 解压缩 Yaw 角度
        int16_t compressed_data = (int16_t)((rx_data[0] << 8) | rx_data[1]);
        motor->received_upyaw = ((float)compressed_data / 32767.0f) * 3.5f;
    }
}
```

#### 步骤 3：底盘跟随控制（底盘板）

```c
// 伪代码示例
void chassis_follow_gimbal_control(chassis_t *chassis)
{
    // 获取底盘自身 Yaw（来自底盘 IMU）
    float chassis_yaw = INS_angle[2];

    // 获取云台 Yaw（来自云台板 CAN）
    float gimbal_yaw = C_data.received_upyaw;

    // 计算相对角度误差
    float angle_error = gimbal_yaw - chassis_yaw;

    // 角度归一化到 [-PI, PI]
    while (angle_error > PI) angle_error -= 2*PI;
    while (angle_error < -PI) angle_error += 2*PI;

    // PID 控制底盘旋转
    chassis->Wz_set = PID_calc(&chassis->turn_pid, angle_error, 0);
}
```

---

## 总结

本项目是 RoboMaster 轮腿机器人的 **云台板控制系统**，负责云台稳定、射击控制和视觉自瞄。关键特点：

1. **双板架构**：底盘板通过读取云台 Yaw 电机编码器实现跟随（备用协议 CAN ID 0x15 未启用）
2. **高频控制**：云台任务运行在 1ms 周期（1000Hz），保证高精度姿态稳定
3. **多系统融合**：集成遥控器、裁判系统、视觉上位机，实现完整的作战功能
4. **模块化设计**：行为层 + 任务层双层架构，便于扩展新的控制模式

**重要文件索引**：
- 云台控制：`application/gimbal_task.c` 
- 射击控制：`application/shoot.c`
- 板间通信：`application/CAN_receive.c` (第613-626行：备用通信协议)
- 视觉自瞄：`MDK-ARM/AutoGimbal.c`
- PID 参数：`application/gimbal_task.h`