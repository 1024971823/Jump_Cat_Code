# FreeRTOS 迁移与四轮遥控控制说明

> 记录本次 FreeRTOS 接入 + SBUS 遥控 + 四电机扩展的全部变更，  
> 方便后续维护和调参。

---

## 一、发现的问题及修复

### 问题 1（已修复）：`Task_Loop()` 是不可达死代码
**文件**：`Core/Src/main.c`  
**原因**：`osKernelStart()` 在内核正常启动后永不返回，其后的 `while(1){ Task_Loop(); }` 永远不会执行，但它引用了 `Delay_Millisecond`（忙等待函数）；保留容易误导新开发者以为主循环还在这里运行。  
**修复**：将 `Task_Loop()` 替换为注释，说明此处为 RTOS 启动失败的安全死循环。

---

### 问题 2（已修复）：`monitor_task` 栈空间仅 128 字（512 B），严重不足
**文件**：`Core/Src/freertos.c`  
**原因**：CubeMX 默认给了 128 × 4 = 512 Byte 栈；该任务未来会调用 USB CDC 输出、浮点运算等，极易发生栈溢出（运行时 Hard Fault，且 `configCHECK_FOR_STACK_OVERFLOW = 2` 会捕获到）。  
**修复**：将 `monitor_taskBuffer` 从 128 字扩充为 **512 字（2 KB）**。

---

### 问题 3（已修复）：三个 RTOS 任务全部是空占位（只有 `osDelay(1)`）
**文件**：`Core/Src/freertos.c`  
**原因**：CubeMX 只生成框架，任务体需手动填充。  
**修复**：见下文"新增功能"。

---

### 问题 4（已修复）：STW 电机存活检测在 TIM7 ISR 中，与 ctrl_task 的 CAN 发送存在竞争
**文件**：`User_File/4_Task/tsk_config_and_callback.cpp`  
**原因**：`TIM_100ms_Alive_PeriodElapsedCallback()` 内部会调用 `CAN_Send_Enter()`（发送 CAN 使能帧），而 ctrl_task 也在 1ms 不断调用 `TIM_Calculate_PeriodElapsedCallback()`（发送控制帧）。两者都调用 `HAL_FDCAN_AddMessageToTxFifoQ()`，若 TIM7 ISR 打断 ctrl_task 中的 CAN 调用，可能破坏 TX FIFO 操作。  
**修复**：从 `Task1ms_Callback`（TIM7 ISR）中移除 STW 电机存活检测，移至 `ctrl_task` 内每 100ms 执行一次，所有 STW CAN 操作集中在同一任务上下文，消除并发冲突。

---

### 问题 5（已修复）：固定力矩测试代码（-5 Nm）留在 ISR 中会干扰正常控制
**文件**：`User_File/4_Task/tsk_config_and_callback.cpp`  
**原因**：遗留的单电机力矩测试代码 `motor_stw.Set_Target_Torque(-5.0f)` 在 TIM7 每 5ms 触发一次，如果新的 ctrl_task 同时设置目标速度，两者相互覆盖导致行为不可预测。  
**修复**：完全删除该测试块。

---

### 问题 6（已修复）：`i6x.c` 原始代码 Bug —— failsafe/frame_lost 解析在 `#if MAPPING_ENABLE` 块内
**文件**：`User_File/2_Device/Remote/i6x.c`（新创建）  
**原因**：原始提供的代码将 `frame_lost` 和 `failsafe` 标志位的解析写在了 `#if MAPPING_ENABLE ... #endif` 条件编译块内，若关闭映射则永远读不到失控信号。  
**修复**：将 failsafe/frame_lost 解析移至 `#if` 块外，确保无论通道映射开关状态如何都能正确检测失控。

---

## 二、新增内容

### 2.1 SBUS 遥控解析文件
| 文件                              | 说明                       |
| --------------------------------- | -------------------------- |
| `User_File/2_Device/Remote/i6x.h` | FS-i6x 数据结构、接口声明  |
| `User_File/2_Device/Remote/i6x.c` | SBUS 25 字节解帧、通道映射 |

> CMake 会自动扫描 `User_File/**/*.{c,h}`，无需手动修改 CMakeLists.txt，  
> 但**需要重新运行 CMake Configure**（见第三节）。

**通道定义**：
```
ch[0] = 右摇杆左右 (Aileron,   ±660)
ch[1] = 右摇杆上下 (Elevator,  ±660)  ← 前进/后退
ch[2] = 左摇杆上下 (Throttle,  ±660)
ch[3] = 左摇杆左右 (Rudder,    ±660)  ← 左转/右转
ch[4] = 旋钮VRA    (±660)
ch[5] = 旋钮VRB    (±660)
s[0]~s[2] = 两档拨杆 (-1/+1)
s[3]       = 三档拨杆 (-1/0/+1)
```

---

### 2.2 四个 8115-36 电机扩展

| 索引           | CAN ID | 位置 |
| -------------- | ------ | ---- |
| `motor_stw[0]` | 0x01   | 左前 |
| `motor_stw[1]` | 0x02   | 左后 |
| `motor_stw[2]` | 0x03   | 右前 |
| `motor_stw[3]` | 0x04   | 右后 |

**控制模式**：`Motor_STW_Control_Method_OMEGA`（速度闭环）  
**CAN 路由**：所有电机回复帧 CAN ID 均为 `0x00`，`Data[0]` 内嵌 Motor ID。`CAN1_Callback` 中对四个对象全部调用 `CAN_RxCpltCallback()`，各对象内部在 `Data_Process()` 中自行过滤。

---

### 2.3 FreeRTOS 任务结构

```
osKernelStart()
    │
    ├─ defaultTask  [Normal]  1ms
    │   └─ MX_USB_DEVICE_Init() → osDelay(1) 循环
    │
    ├─ ctrl_task    [High]    1ms  ← 核心控制
    │   └─ RTOS_Ctrl_Task_Loop()
    │       ├─ 原子读取 i6x_ctrl (PRIMASK 保护)
    │       ├─ Failsafe 检测 → 停车
    │       ├─ 死区 → 差速混控
    │       ├─ Set_Target_Omega × 4
    │       ├─ TIM_Calculate_PeriodElapsedCallback × 4 (PID + CAN 发送)
    │       └─ [每100ms] TIM_100ms_Alive × 4
    │
    ├─ remote_task  [AboveNormal] 20ms
    │   └─ RTOS_Remote_Task_Loop() ← 预留：拨杆模式切换
    │
    └─ monitor_task [Low]     50ms
        └─ 预留：调试输出

TIM7 ISR (1ms, 独立于 RTOS Tick)
    ├─ LED / Buzzer / Key 处理
    ├─ DJI motor alive check (每100ms)
    ├─ BMI088 (每128ms)
    ├─ VOFA USB 输出
    ├─ TIM_1ms_CAN (DJI 电机 CAN 发送)
    └─ TIM_1ms_IWDG (喂狗 ← 必须保留在 ISR，不能放到任务)
```

> **SysTick 冲突已解决**：  
> CubeMX 自动生成了 `stm32h7xx_hal_timebase_tim.c`，将 HAL 时基从 SysTick 切换到 **TIM13**，FreeRTOS 独占 SysTick，互不干扰。

---

### 2.4 差速混控公式

$$
v = ch_1 / 660 \times \omega_{max}
$$
$$
\omega = ch_3 / 660 \times \omega_{max}
$$
$$
\omega_{left} = v + \omega, \quad \omega_{right} = v - \omega
$$

右侧电机因物理安装方向相反取反后下发。若实测前进时某侧反转，修改 `RTOS_Ctrl_Task_Loop` 中对应的正负号即可。

---

## 三、如何操作（上手步骤）

### 步骤 1：确认 SBUS 信号接线

FS-i6x 搭配 iA6B 接收机，**SBUS 信号是反相 TTL**。有两种方案：
- **方案 A（推荐）**：在接收机 SBUS 输出和 UART5_RX 引脚之间串接一个 74HC14（施密特反相器）。
- **方案 B**：在 CubeMX 中打开 UART5 → Advanced Features → **RX pin active level inversion**，然后重新生成代码。

> UART5 已配置为 **100000 baud, 9-bit, Even parity, 2 Stop bits, RX-only**，与 SBUS 协议完全匹配。

---

### 步骤 2：确认电机 ID

用上位机（如达妙 MasterTool 或 ODrive GUI）将四个 8115-36 的 CAN ID 分别设置为 **0x01 / 0x02 / 0x03 / 0x04**。电机内部存储，断电保持。

---

### 步骤 3：重新运行 CMake Configure

因为新增了 `User_File/2_Device/Remote/i6x.h` 和 `i6x.c`，需要刷新 CMake 的 GLOB 列表：

```bash
# 在 VS Code 中：
# Ctrl+Shift+P → CMake: Configure
# 或命令行：
cd build/Debug
cmake ../..
```

---

### 步骤 4：编译 & 烧录

```bash
# VS Code 任务 "CMake: build" 或：
cmake --build build/Debug --parallel
```

---

### 步骤 5：调参顺序（重要！）

1. **单轮验证**：先上电只连一个电机（0x01），遥控前推，观察转向是否正确。
2. **双侧验证**：同时连左侧两轮（0x01, 0x02），前进时应同向、同速。
3. **四轮直行**：四轮全接，直线行驶不跑偏。
4. **转向验证**：左摇杆拨左/右，确认原地/转向行为符合预期。
5. **Failsafe 验证**：关闭遥控器，确认机器人 2-3 秒内停车（等待 `TIM_100ms_Alive` 检测到离线后 `ctrl_task` 进入 failsafe 停车）。

> **PID 参数**（`tsk_config_and_callback.cpp` Task_Init 中）初始值：  
> `PID_Omega.Init(Kp=0.5, Ki=0, Kd=0, DB=0, IntMax=5, OutMax=18)`  
> 若轮子抖动 → 降低 Kp；若追踪慢 → 适量增加 Kp 或加微小 Ki。

---

## 四、文件变更总览

| 文件                                           | 类型     | 主要变更                                                   |
| ---------------------------------------------- | -------- | ---------------------------------------------------------- |
| `User_File/2_Device/Remote/i6x.h`              | **新建** | SBUS 数据结构 & 接口                                       |
| `User_File/2_Device/Remote/i6x.c`              | **新建** | SBUS 解帧（含 Bug 修复）                                   |
| `User_File/4_Task/tsk_config_and_callback.h`   | **修改** | 新增 `RTOS_Ctrl_Task_Loop` / `RTOS_Remote_Task_Loop` 声明  |
| `User_File/4_Task/tsk_config_and_callback.cpp` | **修改** | 四电机数组、SBUS 回调、CAN 分发、删测试代码、RTOS 任务实现 |
| `Core/Src/freertos.c`                          | **修改** | include 添加、monitor 栈扩容、三个任务体实现               |
| `Core/Src/main.c`                              | **修改** | 删除 `osKernelStart()` 后不可达的 `Task_Loop()` 调用       |

---

## 五、注意事项

1. **不要在 CubeMX 重新生成代码时覆盖 USER CODE 区域**  
   所有修改均写在 `/* USER CODE BEGIN ... */` / `/* USER CODE END ... */` 之间，CubeMX 重新生成会保留这些区域。  
   **例外**：`freertos.c` 中 `monitor_taskBuffer[512]` 不在 USER CODE 区域，每次重新生成会被还原为 128——需要重新改回 512。

2. **IWDG 喂狗必须保持在 TIM7 ISR**  
   `TIM_1ms_IWDG_PeriodElapsedCallback()` 仍在 TIM7 ISR（`Task1ms_Callback`）中调用，无论 RTOS 任务是否调度，看门狗都会被定时喂养。切勿将其移入任务。

3. **DJI GM6020 测试代码保留**  
   `motor.Set_Target_Angle(PI)` 和 `motor.TIM_Calculate_PeriodElapsedCallback()` 仍保留在 `Task1ms_Callback` 中（原有测试代码），可按需删除。

4. **右侧电机方向**  
   `motor_stw[2]` 和 `motor_stw[3]` 的目标速度取了负号（`-right_omega`）。这是基于"右侧电机安装方向与左侧相反"的假设。实测若不符，修改 [tsk_config_and_callback.cpp](User_File/4_Task/tsk_config_and_callback.cpp) 中 `RTOS_Ctrl_Task_Loop` 对应正负号即可。
