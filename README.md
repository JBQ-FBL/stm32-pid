# Pid — 直流电机速度闭环

STM32F103C8T6 + TB6612 + JGB37-520 编码器电机，**5 ms 控制周期**做速度环 PID。
目标：阶跃响应 **< 50 ms**、静差 **< 5%**、扰动后能自己回到目标。

> 接线依据：`../docs/wiring-plan.md`　｜　考核要求：`~/stm32/docx/`　｜　工具链踩坑：`~/stm32/AGENTS.md`

---

## 1. 硬件

| 器件 | 型号 / 参数 | 用途 |
|---|---|---|
| 主控 | STM32F103C8T6 最小系统板（LQFP48，64 KB Flash / 20 KB RAM） | 控制 |
| 电机驱动 | TB6612FNG（HW-048 蓝板，**16 脚，无 STBY**） | 单路 A 通道，PWM 调速 + 方向 |
| 电机 | JGB37-520 直流减速电机 12 V / **333 rpm**（输出轴），带霍尔编码器 | 被控对象 |
| 编码器 | 11 PPR（电机轴）× 4 倍频 × 减速比 1:30 → **理论 1320 计数/圈**（**待标定**） | 测速反馈 |
| 显示 | 0.96" OLED，SSD1306，I2C | 显示目标/实际/参数 |
| 串口 | CH340 USB-TTL | VOFA+ 看曲线 + 收命令 |
| 调试器 | 克隆版 ST-Link V2（**无虚拟串口**） | SWD 烧录/调试 |
| 电源 | 3×18650 串联（标称 11.1 V，满电 12.6 V） | 只给 TB6612 `VM` 和电机 |

**其他**：18650 电池盒、杜邦线（尽量 <15 cm）、面包板、电机支架。

---

## 2. 接线

### 2.1 电源与共地（不共地 = 一切诡异现象的源头）

```
 电池 + ── 开关 ────────────────► TB6612 VM    ← 11.1 V 只走这一根线，不上面包板
 电池 − ──┐
          │
 ┌────────┴──── 面包板「功率地段」（红/蓝轨左半边）────────┐
 │   电池 −          TB6612 GND                          │
 └────────────────────────┬───────────────────────────────┘
                          │ ★ 只在这一点用短粗线相连（星点）
 ┌────────────────────────┴── 面包板「逻辑地段」（右半边）──┐
 │ 开发板 GND  ST-Link GND  CH340 GND  OLED GND  编码器黑线 │
 └──────────────────────────────────────────────────────────┘

 开发板 3.3V ──┬── TB6612 VCC        ← 三条都吃这一路（合计 60~80 mA）
               ├── 编码器 蓝线
               └── OLED VCC
```

- 开发板由 **ST-Link 的 3.3 V** 供电；**不要**把电池接到开发板 5V/3.3V。
  ⚠️ **板子的 USB 和 ST-Link 3V3 二选一**：3.3 V 排针是直连 MCU 的（不经 AMS1117），两个源同时接会互相倒灌。
- **11.1 V 不上面包板**：面包板上常有 3.3 V 器件，一根错跳线就烧 MCU / OLED / 霍尔。电池正 → 开关 → `VM` 一根线直连。
- **功率地与逻辑地分段、只在一点相连**：电机回流（1~3 A）只能走「电池− ↔ TB6612 GND」那段，不许穿过逻辑地段，否则地弹直接进编码器 A/B。
  很多人以为两条蓝轨是通的，但 **830 孔面包板的电源轨中间通常断开** —— 先用万用表通断档量一下。
- **编码器黑线接逻辑地段靠近 MCU 那一头**，别和 TB6612 GND 共用一个孔；电机那三段线用短粗线、插到底（堵转 1~3 A，触点会热）。
- TB6612 `VM` 操作范围 **2.5~13.5 V**（绝对最大 15 V）→ **3S 满电 12.6 V 可以，4S（16.8 V）绝对不行**；余量只有 0.9 V，`VM`–`GND` 就近并 **100 µF + 100 nF**。
- 逻辑 `VCC` 接 3.3 V（芯片 2.7~5.5 V，`VIH` 只要 2.31 V），和 MCU 同电平。**VCC 绝不能接 11.1 V**。
- 3.3 V 这一路总负载约 **60~80 mA**（MCU 30~40 + OLED 10~20 + 霍尔 10~20），ST-Link 3V3 余量不大；
  全开时用万用表量开发板 3.3 V 排针，**低于 3.2 V 就是掉压**（症状：OLED 花屏、测速乱跳）。

### 2.2 TB6612（HW-048，16 脚）

| TB6612 | 接到 | 说明 |
|---|---|---|
| `VM` | 电池 +（经开关） | 左排倒数第 2 脚 |
| `GND`（任一） | 公共地 | 左排最下面，和 VM 紧挨着 |
| `VCC` | 开发板 3.3 V | 逻辑电源 |
| `PWMA` | **PA8**（TIM1_CH1，20 kHz） | 右排最上面 |
| `AIN1` | **PA4** | ⚠️ 右排是 **AIN2 在上、AIN1 在下**，别按名字顺序插 |
| `AIN2` | **PA5** | |
| `AO1` / `AO2` | 电机 **红 / 白** | 对调 = 反转（优先在代码里改方向，不拆线）|
| `STBY` | **本模块没有这个脚** | 说明书原文「本模块没有 STBY 功能」，16 脚里确实没有 → 不用找、不用接 |
| `BO1/BO2/BIN1/BIN2/PWMB/PWMC` | 不接 | 只用单电机 |

真值表：`AIN1/AIN2 = 1/0` 正转，`0/1` 反转，`0/0` 滑行，`1/1` 刹车。

### 2.3 编码器 6 根线（`../docx/编码器连接图.jpg`）

| 线色 | 含义 | 接到 | 接错的后果 |
|---|---|---|---|
| 红 | 电机电源 + | TB6612 `AO1` | 这是功率线，接 3.3 V 会短路或电机不转 |
| 白 | 电机电源 − | TB6612 `AO2` | 同上；红白对调 = 反转（无害）|
| 蓝 | 编码器电源 + | 开发板 **3.3 V** | ⚠️ 接 11.1 V 直接烧霍尔：电机还能转，但永远没反馈 |
| 黑 | 编码器 GND | 开发板 GND | 不接地 = 读数无效 |
| 黄 | C1 / A 相 | **PA6**（TIM3_CH1） | 方向反 → PID 变正反馈 → 飞车 |
| 绿 | C2 / B 相 | **PA7**（TIM3_CH2） | 同上 |

这 6 根在同一个 PH2.0 插头上，**插之前逐根对一遍**。12 V 电机脉冲约 7.3 kHz，杜邦线要短。

### 2.4 CubeMX 配置（按左边栏分类，照这个顺序点）

> 等宽字体的字就是 CubeMX 界面上**字段名 / 选项的原文**，按这个名字找就能对上。

**① System Core**

**GPIO**（System Core → GPIO → 点引脚，右侧面板就是下面这 5 项）✅

| 引脚 | 用途 | `GPIO output level` | `GPIO mode` | `GPIO Pull-up/Pull-down` | `Maximum output speed` | `User Label` |
|---|---|---|---|---|---|---|
| **PA4** | 方向 AIN1 | `Low` | `Output Push Pull` | `No pull-up and no pull-down` | `High` | `AIN1` |
| **PA5** | 方向 AIN2 | `Low` | `Output Push Pull` | `No pull-up and no pull-down` | `High` | `AIN2` |
| **PB8** | OLED 时钟 | `High` | `Output Open Drain` | `No pull-up and no pull-down` | `High` | `OLED_SCL` |
| **PB9** | OLED 数据 | `High` | `Output Open Drain` | `No pull-up and no pull-down` | `High` | `OLED_SDA` |
| **PC13** | 板载 LED | `High`（= 灭）| `Output Push Pull` | `No pull-up and no pull-down` | `Low` | `LED` |

- 为什么低电平亮：PC13 硬件是 LED 阳极接 3.3 V、阴极接 PC13，所以 `Low` = 点亮。
- PB8/PB9 也可以**完全不配**（驱动会自己 init 成开漏），但配了就必须是 `Output Open Drain`，不能推挽。

**NVIC**（System Core → NVIC，在列表里找 `TIM2 global interrupt`）✅

- `TIM2 global interrupt` = ☑ 勾上；`Preemption Priority` = **1**，`Sub Priority` = **0**
- 其余保持默认：SysTick = 15；**USART1 的全局中断不勾**（串口发送在主循环，不在中断里等）
- 为什么：控制中断必须能抢占一切，否则 dt 会被拉长，PID 的积分/微分全歪
- ⚠️ **不勾的后果是静默的**：`HAL_TIM_Base_Start_IT()` 照样返回 OK，但中断永远不进，程序看起来像“卡住”；
  验证：`stm32f1xx_it.c` 里应该有 **`void TIM2_IRQHandler(void)`**，`stm32f1xx_msp.c` 里应该有 `HAL_NVIC_EnableIRQ(TIM2_IRQn)`。
  两个都没有 = 没勾。（`nm` 里能看到的那个 `TIM2_IRQHandler` 是 `W`=weak，启动文件的死循环桩，不算）

**RCC**（System Core → RCC）✅

- `High Speed Clock (HSE)` = `Crystal/Ceramic Resonator` → PD0/PD1 自动变成 `OSC_IN` / `OSC_OUT`

**SYS**（System Core → SYS）✅

- `Debug` = `Serial Wire`（⚠️ 选 `Disable` 会把 SWD/JTAG 全关掉，程序一跑芯片就连不上，只能改 BOOT0 救砖）
- `Timebase Source` 保持 `SysTick`

**② Timers**

**TIM1 → PA8（电机 PWM，20 kHz）** ✅

- `Timers → TIM1 → Channel1` 选 `PWM Generation CH1`（下拉里排第 **10** 项，在 `PWM Generation No Output` 后面）
- `Clock Source` = `Internal Clock`；`Combined Channels` = `Disable`
- `Parameter Settings`：`Prescaler` = **0**、`Counter Mode` = `Up`、`Counter Period (ARR)` = **3599**、`auto-reload preload` = `Enable`（其余默认）
- `PWM Generation Channel 1`：`Mode` = `PWM mode 1`、`Pulse` = **0**、`Output compare preload` = `Enable`、`CH Polarity` = `High`
- 引脚 PA8 自动变成 `AF Push Pull`；`Maximum output speed` 改成 **`High`**（CubeMX 默认给 `Low`），`User Label` 填 `MOTOR_PWM`
  ℹ️ F1 的 GPIO 只有 **`Low`(2 MHz) / `Medium`(10 MHz) / `High`(50 MHz)** 三档，**没有 `Very High`**（那是 F4/F7 的选项）
- 为什么：72 MHz ÷ (3599+1) = **20 kHz**；`Pulse` 就是占空比（0 = 输出全低）

**TIM2 → 不占引脚（5 ms 控制时基）** ✅

- `Clock Source` = **`Internal Clock`**（这一步才叫启用 TIM2）
- 四个通道全部 `Disable`；`Combined Channels` = `Disable` → **芯片图上一个引脚都不会变绿**（变了 = 点错通道了）
- `Parameter Settings`：`Prescaler` = **7199**、`Counter Period (ARR)` = **49**（其余默认）
  ⚠️ **默认值是 65535，必须手动改成 49**；忘了就是 **6.55 s** 一次中断，不是 5 ms
- `NVIC Settings` 页签：`TIM2 global interrupt` ☑
- 为什么：72 MHz ÷ 7200 = 10 kHz，数 **50** 个计数 = **5 ms**

**TIM3 → PA6 / PA7（编码器，4 倍频）** ✅

- `Timers → TIM3 → Combined Channels` 选 `Encoder Mode`
- `Parameter Settings`：`Encoder Mode` = **`Encoder Mode TI1 and TI2`** —— ⚠️ **不能选 `Encoder Mode TI1`**，
  那是 2 倍频，计数只有一半，所有 rpm 都会偏小 2 倍（标定怎么算都对不上）
- 同页：`Prescaler` = **0**、`Counter Period` 保持 **65535**（满量程，软件用 `int16_t` 差值处理回绕）
- **`IC1 Filter` = 6 且 `IC2 Filter` = 6**（CubeMX 只把 IC1 默认填 6，**IC2 默认是 0**，要手动改）；
  `IC1 Polarity` = `Rising Edge`、`IC2 Polarity` = `Rising Edge`
- 引脚 PA6 / PA7：**`GPIO Pull-up/Pull-down` 改成 `Pull-up`**（CubeMX 默认 `No pull`），`User Label` 填 `ENC_A` / `ENC_B`
- 为什么：`TI1 and TI2` = A/B 双相双沿 → 每圈计数 ×4；滤波 6 是去掉 7.3 kHz 脉冲上的毛刺

**③ Connectivity**

**USART1 → PA9 / PA10（115200 8N1）** ✅

- `Connectivity → USART1 → Mode` = `Asynchronous`
- `Parameter Settings`：`Baud Rate` = **115200** Bits/s、`Word Length` = `8 Bits`、`Parity` = `None`、`Stop Bits` = `1`、`Flow Control` = `None`（`Data Direction` 保持 `Receive and Transmit`）
- `GPIO Settings`：PA9 = `USART1_TX`（`AF Push Pull` / `Very High`）、PA10 = `USART1_RX`（`Input`）
- `NVIC Settings`：**一个中断都不勾**（发送和解析都放主循环）

**注**

- **冲突已避开**：TIM1 只用 CH1（PA8）→ PA9/PA10 让给 USART1；编码器用 TIM3 不用 TIM1；PWM 与 USART1 不抢脚。
- **PB6 / PB7 不配**（硬件 I²C1 不用，空着备用）；PB8/PB9 当普通 GPIO 做软件 I²C（TIM4 没用到）。
- **为什么 OLED 用 PB8/PB9 而不是 PB6/PB7（硬件 I²C1）**：驱动母本 `oled-ssd1306` 默认就是这两个脚，`oled.h` 一个宏都不用改；F103 的硬件 I²C 还有著名的总线锁死勘误，软件模拟更稳。真要换脚就改 `oled.h` 里 **6 个宏**（4 个引脚 + 2 个端口时钟）。
- **OLED 接线（4 针）**：`VCC → 3.3V`、`GND → GND`、`SCL → PB8`、`SDA → PB9`。SCL/SDA **不交叉**；排针上 `B9 B8 B7 B6` 连着四个孔，但 **B7/B6 是普通 GPIO，既不供电也不接地**。

---

## 3. Clock Configuration + Project Manager

**时钟树**（Clock Configuration 页签）：HSE 8 MHz → PLL ×9 → **SYSCLK 72 MHz**；AHB /1；APB1 /2 = 36 MHz（**定时器 ×2 = 72 MHz**）；APB2 /1 = 72 MHz。
（F103 的 APB 分频器对定时器有 ×2 补偿，所以 TIM1/TIM2/TIM3 都按 72 MHz 算。）

| 项 | 值 |
|---|---|
| 工程名 | `Pid` —— **别改**：`Pid.elf` 被 `.vscode/launch.json` 和烧录脚本引用，改名要三处一起改 |
| Toolchain | **CMake** |
| 位置 | `/home/niko/stm32/PID/Pid` |

**四个换算**（答辩时用得上）

- PWM：72 MHz / (3599+1) = **20 kHz**；占空比统一用千分比 → `CCR = ‰ × 3600 / 1000`，限幅 ±900‰ → CCR ≤ 3240
- 5 ms：72 MHz / 7200 = 10 kHz，每 50 个计数 = **5 ms**；50 ms 指标下有 10 个采样点
- 编码器：11 PPR × 4 × 减速比 30 = **1320 计数/圈**（待标定）；`rpm = Δcounts × 60/(dt×1320)`，dt=5 ms 时 = **Δcounts × 9.09**
- 串口：115200 8N1 = 11 520 B/s；VOFA+ JustFloat 一帧 16 B（3×float + 4 B 尾），100 Hz 只占 **14%** 带宽

`USER CODE BEGIN/END` 以外的区块不要手改 —— 重新生成会被覆盖。

---

## 4. 代码结构

```
Pid/
├── Core/                  CubeMX 生成：main.c / gpio.c / 中断 / 时钟
├── Components/            drv 从 ~/stm32-drivers 装的驱动
│   ├── motor-tb6612/      PWM + 方向 + 编码器测速（HAL）
│   ├── oled-ssd1306/      软件 I2C 显示
│   ├── pid/               pid.c/h：纯 C，无 HAL，可在 PC 上单测
│   ├── speed-loop/        5 ms 调度、测速、限幅、保护、虚拟被控对象
│   ├── vofa/              JustFloat 组帧（只依赖一个 write 回调）
│   └── cmd/               串口命令：S200 / S-100 / X / A / ?
└── tests/                 host 测试（gcc 直接编，不进固件）
```

**铁律**：
- `pid.c`、`vofa.c` 不许 include 任何 HAL 头 → 在 `/tmp` 里就能验证，不等硬件。
- 新增 `.c` 登记**根** `CMakeLists.txt` 的 `target_sources`（`cmake/stm32cubemx/CMakeLists.txt` 是生成物，别动），否则链接报 `undefined reference`。

### CubeMX 会覆盖哪些文件（手改守则）

| 想改什么 | 改哪里 | 重新生成后 |
|---|---|---|
| 外设参数（PSC / ARR / 上下拉 / 波特率…）| **`Pid.ioc`** → 点 `GENERATE CODE` | 保留 |
| 业务代码 | `/* USER CODE BEGIN/END */` 夹层，或 `Components/` 里新模块 | 保留 |
| CubeMX 不给配的（AF 脚内部上拉等）| `USER CODE BEGIN 2` 里再 `HAL_GPIO_Init()` 覆盖 | 保留 |
| 手改 `Core/Src/tim.c` 里 `MX_TIM2_Init()` 的数值 | —— | ❌ 被还原 |
| `cmake/stm32cubemx/CMakeLists.txt`、`.mxproject`、`Drivers/`、`stm32f1xx_hal_conf.h` | —— | ❌ 被重写 |

**CMake 版 CubeMX 的三条规律**（2026-10-20 用 `git diff` 实测，改了 10 个文件 + 新增 `tim.c/usart.c` 一对）

1. **一外设一对文件**：`Core/Src/tim.c` + `Core/Inc/tim.h`（含 `MX_TIMx_Init` / `MspInit` / `MspPostInit`），
   `main.c` 里只剩 `#include` + `MX_*_Init()` 调用 —— **查配置去看外设自己的 .c，不是 main.c**。
2. **AF 脚的 GPIO 配置不在 `gpio.c`**：输入脚在 `HAL_<IP>_MspInit()`、输出脚在 `HAL_<IP>_MspPostInit()`。
   所以 PA6/PA7 的 `Pull-up`、PA8 的 `GPIO_SPEED_FREQ_HIGH` 都在 **`tim.c`** 里，改 `gpio.c` 没用。
3. **勾了 NVIC 才有三件套**：`stm32f1xx_it.c` 的 `TIM2_IRQHandler` + `it.h` 原型 + `MspInit` 里的
   `HAL_NVIC_SetPriority/EnableIRQ`。判据：`arm-none-eabi-nm build/Debug/Pid.elf | grep TIM2_IRQHandler`
   要看到 **`T`**；看到 **`W`** = 还是启动文件的 weak 死循环桩（= 没勾中断）。

### 进度

- [x] CubeMX 骨架 + 72 MHz + SWD，`Pid.elf` 能编出来
- [x] 五个外设已生成代码（TIM1 PWM / TIM2 时基 / TIM3 编码器 / USART1 / GPIO），编译通过
      —— 2026-10-20 实测：FLASH **10208 B / 64 KB（15.6%）**、RAM **1872 B / 20 KB（9.1%）**
- [x] **CubeMX 全部配完并核对**（2026-10-20 16:05 生成；`tim.c` 里 TI12 / IC1&IC2 Filter 6 / Pull-up / High 全部到位）
- [ ] 装驱动（motor-tb6612、oled-ssd1306）+ 拷入 pid / cmd
- [ ] 无电机验证：虚拟被控对象 + VOFA 曲线 + OLED + PC 端指标脚本
- [ ] 真机：编码器标定 → 开环验方向 → 闭环调参 → 记录数字
- [ ] 答辩：扰动实验 + 参数固化 + 讲解稿

---

## 5. 编译 / 烧录 / 调试

```bash
# 编译 + 烧录 + 复位运行（不进调试，不占 ST-Link）—— 日常用这个
bash ~/stm32/scripts/build-flash.sh ~/stm32/PID/Pid

# 只编译
FLASH=0 bash ~/stm32/scripts/build-flash.sh ~/stm32/PID/Pid
```

VS Code：`Ctrl+Alt+F` 同上 · `Ctrl+Alt+B` 只编译 · **`F5` 调试**（openocd，先烧再停在 `main`）。
`F5` 走 `.vscode/launch.json` 里定制的 openocd，因为 ST 官方工具链不认这块克隆版 ST-Link 的旧固件。

判据：烧录输出里出现 **`** Verified OK **`**；`st-info --probe` 里 `chipid: 0x410`、`flash: 64`。

---

## 6. 上电前必查 / 常踩的坑

1. **共地**：电池−、TB6612 GND、开发板 GND、CH340 GND 全连一起。
2. **编码器蓝线只接 3.3 V**，红线白线是电机功率线 —— 接错会烧霍尔或短路。
3. **先验方向再闭环**：`MOTOR_SetDuty(+200)`（20%）时 rpm 必须是**正的**，否则 PID 是正反馈。
4. **PC13 低电平点亮**；`HAL_Delay()` 单位是**毫秒**（写 100000 = 100 秒）。
5. **F5 是调试**（占 ST-Link、停在 `main`，板子灯会僵住）；只想跑用 `Ctrl+Alt+F`。
6. 烧录报端口占用：先 `pgrep -a -x openocd` 确认是「你的调试会话」还是「残留」，是残留才 `pkill -x openocd`。
   别随手杀 —— 强杀会让 ST-Link 卡住，得拔插 USB。
7. probe 连不上先**拔插杜邦线**（尤其 SWDIO/SWCLK 可能丝印反），再怀疑代码。
8. 输出限幅先设 ±300‰ 验证方向，确认无误再放开到 ±900‰；调参全程用同一块电池、同一电量。
