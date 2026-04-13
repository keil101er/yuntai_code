# Pitch IMU Closed-Loop Fix

因为当前工作区只允许写 `MDK-ARM`，不能直接改 `..\application\gimbal_task.c`，这里给出需要手动替换的精确代码。

目的：
- `absolute_angle_set` 继续表示“期望 IMU pitch”
- 下发给 4310 的位置指令改成“当前电机角 + IMU 误差补偿”
- 这样机身前后晃动时，pitch 电机会主动反向补偿，云台 pitch 基本保持不变

在 `..\application\gimbal_task.c` 中，替换下面两个函数。

## 1. `gimbal_motor_auto_angle_control_pitch`

```c
static void gimbal_motor_auto_angle_control_pitch(gimbal_motor_t *gimbal_motor)
{
    fp32 pitch_motor_position_set;
    float min_speed = 5.0f;

    if (gimbal_motor == NULL)
    {
        return;
    }

    if (gimbal_motor->absolute_angle_set > 0.3f)
    {
        gimbal_motor->absolute_angle_set = 0.3f;
    }
    else if (gimbal_motor->absolute_angle_set < -0.45f)
    {
        gimbal_motor->absolute_angle_set = -0.45f;
    }

    angle_error = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);

    // 把 IMU pitch 误差转换成电机位置目标，抵消机身俯仰扰动
    pitch_motor_position_set = rad_format(gimbal_motor->relative_angle + angle_error + 0.02f);
    gimbal_motor->relative_angle_set = pitch_motor_position_set;

    aim_speed = 30.0f * fabs(angle_error) * 5.0f;
    if (aim_speed > 30.0f)
    {
        aim_speed = 30.0f;
    }
    if (aim_speed < min_speed && fabs(angle_error) > 0.01f)
    {
        aim_speed = min_speed;
    }

    CAN_cmd_4310pitch_pvmode(pitch_motor_position_set, aim_speed);
}
```

## 2. `gimbal_motor_absolute_angle_control_pitch`

```c
static void gimbal_motor_absolute_angle_control_pitch(gimbal_motor_t *gimbal_motor)
{
    fp32 pitch_motor_position_set;

    if (gimbal_motor == NULL)
    {
        return;
    }

    if (gimbal_motor->absolute_angle_set > 0.22f)
    {
        gimbal_motor->absolute_angle_set = 0.22f;
    }
    else if (gimbal_motor->absolute_angle_set < -0.4f)
    {
        gimbal_motor->absolute_angle_set = -0.4f;
    }

    angle_error = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);

    // 用 IMU 误差修正当前电机角，而不是把 IMU 目标直接当成电机位置发出去
    pitch_motor_position_set = rad_format(gimbal_motor->relative_angle + angle_error);
    gimbal_motor->relative_angle_set = pitch_motor_position_set;

    aim_speed = 30.0f * fabs(angle_error) * 0.6f;
    if (aim_speed > 30.0f)
    {
        aim_speed = 30.0f;
    }
    else if (aim_speed < 0.0f)
    {
        aim_speed = 0.0f;
    }

    CAN_cmd_4310pitch_pvmode(pitch_motor_position_set, 10.0f);
}
```

## 核心变化

旧逻辑：

```c
CAN_cmd_4310pitch_pvmode(gimbal_motor->absolute_angle_set, ...);
```

这相当于把“IMU pitch 目标”直接当成“电机位置目标”，机身俯仰时会把扰动带到枪管上。

新逻辑：

```c
angle_error = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
pitch_motor_position_set = rad_format(gimbal_motor->relative_angle + angle_error + bias);
```

这会根据当前 IMU pitch 误差去修正电机位置，所以机身前后晃时，pitch 会主动反向补偿。

## 调试建议

- 先看 `pitch_relative_set_1000` 和 `pitch_relative_angle_1000`，确认机身晃动时两者会出现补偿差值
- 再看 `pitch_ins_set_1000` 和 `pitch_ins_int_1000`，确认 IMU pitch 能基本贴住设定值
- 如果补偿方向反了，把下面一行改成减号：

```c
pitch_motor_position_set = rad_format(gimbal_motor->relative_angle + angle_error);
```

改为：

```c
pitch_motor_position_set = rad_format(gimbal_motor->relative_angle - angle_error);
```

