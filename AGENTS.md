# Workspace Notes

This repository is the gimbal-board side of a dual-board robot.

## Scope

- Treat this codebase as the upper controller for gimbal, auto-aim, shooting, INS, referee, and UI.
- Do not assume chassis motion code in this repository is running.
- Chassis-related logic may still appear as shared framework code or as board-to-board communication interfaces.

## Actual Runtime Tasks

The active FreeRTOS tasks are defined in `Src/freertos.c`.

Running tasks include:
- `test_task`
- `calibrate_task`
- `gimbal_task`
- `INS_task`
- `led_RGB_flow_task`
- `referee_usart_task`
- `usb_task`
- `battery_voltage_task`
- `user_task`

Not running in this project state:
- `chassis_task` is commented out in `Src/freertos.c`

## Files To Read First

For gimbal control and auto-aim, start here:
- `application/gimbal_task.c`
- `application/gimbal_task.h`
- `application/gimbal_behaviour.c`
- `application/gimbal_behaviour.h`
- `MDK-ARM/AutoGimbal.c`
- `MDK-ARM/AutoGimbal.h`

Then follow supporting paths:
- `application/CAN_receive.c`
- `bsp/boards/CANdata_analysis.c`
- `bsp/boards/CANdata_analysis.h`
- `application/shoot.c`

## Control Ownership

- This board executes the gimbal control loop in `gimbal_task`.
- This board receives operator/chassis-board commands through CAN board-to-board communication.
- This board also exchanges data with the vision/NUC side over `UART1` DMA in `AutoGimbal.c`.

## Gimbal Main Loop

The main control loop is in `application/gimbal_task.c`.

Core order each cycle:
1. `gimbal_set_mode()`
2. `gimbal_mode_change_control_transit()`
3. `gimbal_feedback_update()`
4. `gimbal_set_control()`
5. `gimbal_control_loop()`
6. send motor commands and friction-wheel commands

## Behaviour Modes

The gimbal behaviour state machine is in `application/gimbal_behaviour.c`.

Current effective mapping from lower-board `receive_chassis_data.mode_flag`:
- `0` -> `GIMBAL_ZERO_FORCE`
- `1` -> `GIMBAL_ABSOLUTE_ANGLE`
- `2` -> `GIMBAL_AUTO`

Important: several historical modes and branches remain in the file but are commented out and are not part of the current active path.

## Manual Gimbal Control

Manual control currently uses the board-to-board CAN payload parsed into:
- `receive_chassis_data.receive_yaw_ch`
- `receive_chassis_data.receive_pitch_ch`

In `GIMBAL_ABSOLUTE_ANGLE`, the current active mapping is:
- `yaw = -receive_yaw_ch * 4`
- `pitch = -receive_pitch_ch * 3`

The motors then run in gyro/absolute-angle control with cascaded angle-loop and speed-loop PID.

## Auto-Aim Path

Vision communication is implemented in `MDK-ARM/AutoGimbal.c`.

Current protocol notes:
- RX frame length: `29`
- RX header/tail: `5A A5 ... 7F FE`
- TX frame length: `43`
- UART: `USART1` with DMA + IDLE interrupt

Parsed vision target data lands in `RresPi.Rec`, and `gimbal_control.gimbal_AUTO_ctrl` points to it.

Current active auto-aim behavior:
- `vision_last_target_time` is used to detect fresh targets
- fresh target window is `100 ms`
- yaw and pitch target errors are low-pass filtered before use
- auto mode feeds angle error into `GIMBAL_MOTOR_AUTO`

## Dual-Board Communication Boundary

Do not confuse "chassis code exists in repo" with "chassis task executes here".

What is active:
- lower board sends CAN command packet to this board
- this board parses it into `receive_chassis_data`
- this board may send trigger/fire state back to the lower board with `CAN_gimbal_send__to_chassis()`

What is not active here:
- local `chassis_task`
- local chassis motion control loop

## Current Implementation Cautions

- `application/gimbal_behaviour.c` contains older fallback logic for losing visual targets, but much of it is commented out.
- In the current active path, `auto_flag` is effectively kept enabled while in auto mode, and the system tends to hold the last target angle when no fresh frame arrives.
- `last_auto_data` is initialized in `gimbal_init()`, but any logic depending on it should be re-checked before assuming it is effective.

## Working Rule For Future Changes

- When modifying gimbal or auto-aim behavior, verify the actual runtime path from `gimbal_task` instead of assuming commented framework code is active.
- Treat chassis-related files as low priority unless the change touches board-to-board communication.
