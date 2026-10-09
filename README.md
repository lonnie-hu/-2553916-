# STM32 C Board

本仓库是基于 STM32F407 的 C 板固件工程，包含 FreeRTOS 任务、BMI088 姿态解算、DBus 遥控器接收、CAN 电机控制和调试数据绘图。`c_board/sp_middleware` 使用 `sp::` 命名空间，提供硬件驱动和常用控制算法。

## 主要任务

应用入口位于 `c_board/applications`，当前主要功能如下：

- `imu_task`：通过 SPI 读取 BMI088，使用 Mahony 算法计算 roll、pitch 和连续 yaw，并以约 2 ms 周期更新姿态数据。
- `remote_task`：通过 USART3 接收 DBus 遥控器数据，并在 DMA/空闲接收回调中刷新遥控状态。
- `motor_task`：通过 CAN 控制两台 GM6020（A 为 ID 2，B 为 ID 1）。任务包含位置外环 P、速度内环 PI、拨杆档位、姿态联动、手动拖动和 R 标复位。
- `plot_task`：通过 USART1 周期发送 roll、pitch、连续 yaw，便于上位机绘图或串口调试。
- `led_task`：驱动板载 RGB 灯进行渐变指示。
- `buzzer_task`：启动时鸣叫三次，表示蜂鸣器任务已运行。

电机控制的数据流为：

```text
BMI088 / 遥控器 / 电机反馈
              -> 联动目标角度
              -> 位置环与速度环 PID
              -> CAN 力矩指令
              -> 两台 GM6020
```

更详细的姿态联动、标定和手动拖动说明见 [`c_board/docs/attitude_linkage.md`](c_board/docs/attitude_linkage.md)。

## 工程结构

```text
c_board/
├── applications/       # FreeRTOS 应用任务
├── sp_middleware/      # 硬件驱动、电机驱动和控制算法
├── cmake/              # ARM GCC 工具链和 CubeMX CMake 配置
├── cboard.ioc          # STM32CubeMX 工程配置
├── openocd.cfg         # OpenOCD 下载和调试配置
└── CMakePresets.json   # Debug / Release 构建预设
```

## 构建与下载

需要准备：

- VS Code，以及 CMake Tools、Cortex-Debug 等扩展；
- CMake 和 Ninja；
- `arm-none-eabi-gcc` 工具链；
- OpenOCD（仅下载或调试时需要）。

在 VS Code 中打开 `c_board` 文件夹，选择 CMake 预设 `Debug` 或 `Release` 后执行构建。也可以在 PowerShell 中运行：

```powershell
cd c_board
cmake --preset Debug
cmake --build --preset Debug
```

构建产物位于 `c_board/build/Debug/`。工程已提供 VS Code 任务：

- `CMake: build`：编译固件；
- `OpenOCD: flash`：编译后通过 `openocd.cfg` 下载、校验并复位。

下载前请确认 ST-Link、OpenOCD 连接正常，并确认目标板供电稳定。

## 上电与操作注意事项

### 首次使用前

1. 先断开电机动力电源或让电机处于安全的机械状态，确认 CAN 线、电机 ID 和电源极性正确。
2. 确认 BMI088 安装方向与 `applications/imu_task.cpp` 中的坐标变换矩阵一致；安装方向改变后需要重新检查姿态方向。
3. 确认 GM6020 使用当前代码对应的电流控制协议。代码发送的是 CAN ID `0x1FE`，输出单位为 N·m；不要只修改电机枚举而不重新检查协议、ID 和控制参数。
4. 首次调试时建议卸下可能造成夹伤的机构，限制电机输出并准备急停或断电措施。

### R 标机械标定

标定值只保存在 RAM 中，重启后需要重新标定：

1. 右拨杆置下档，使两台电机不输出主动保持力矩。
2. 手动将 C 板和两台电机的三个 R 标方向对齐。
3. 保持不动，按 C 板 RESET 键（也可以断电重启）。
4. 重启后继续保持对齐至少 2 秒，让程序在下档静止状态下记录 `yaw_R`、`angle_A_R` 和 `angle_B_R`。
5. 切换到右中档测试联动，再切换到右上档测试复位。

如果首次采样时没有对齐，复位方向会错误；应重新对齐并再次 RESET。未完成标定时，上档复位不会驱动电机。

### 拨杆功能

- 右下档：失能，两台电机输出零力矩；
- 右中档：启用姿态联动；
- 右上档：使用已保存的 R 标进行复位；
- 左拨杆：选择联动比例，当前为下档 `0.5`、中档 `-1`、上档 `3`。

手动拖动只应在确认电机输出已停止、机构不会突然运动时进行。失联、姿态数据异常或电机反馈超时后，程序会关闭电机力矩输出；恢复控制前应先确认机构处于安全位置。

## 修改与调试建议

- 主要控制参数位于 `c_board/applications/motor_task.cpp` 文件顶部，包括位置环 Kp、速度环 Kp/Ki、速度限幅和力矩限幅。
- 修改 BMI088 采样周期、坐标方向、CAN 电机 ID 或遥控器串口后，应同时检查 `cboard.ioc`、`CMakeLists.txt` 和对应 HAL 初始化代码。
- `plot_task` 使用 USART1 输出调试数据；连接上位机时请确认波特率、串口电平和共地，避免把调试串口误接到动力接口。
- `motor_task.cpp` 中的 `MOTOR_TASK_TEST` 仅用于电脑端控制算法测试，不应在正常 STM32 固件编译时启用。

## 中间件

同济大学 SuperPower 战队 25 赛季电控中间件，命名空间为 `sp::`。

- `io`：LED、蜂鸣器、BMI088、DBus、CAN 等硬件驱动；
- `motor`：RM 电机、CyberGear 等电机驱动；
- `tools`：Mahony、PID、云台和其他控制算法；
- `referee`：裁判系统协议和 UI 工具。

部分模块的使用示例位于各目录下的 `readme.md`。
