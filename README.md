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
3×18650 (11.1V) ──┬── TB6612 VM        ← 只有电机功率走这里
                  └── GND ─┬── TB6612 GND ─┬── 开发板 GND ─┬── CH340 GND
                                    （所有 GND 必须连成一点）
ST-Link 3.3V ── 开发板 3.3V ──┬── TB6612 VCC（逻辑电源）
                              └── 编码器 蓝线（霍尔电源）
```

- 开发板由 **ST-Link 的 3.3 V** 供电；**不要**把电池接到开发板 5V/3.3V。
- TB6612 `VM` 绝对最大 15 V → **3S 满电 12.6 V 可以，4S（16.8 V）绝对不行**。
- 逻辑 `VCC` 接 3.3 V（芯片支持 2.7~5.5 V），这样和 MCU 同电平。**VCC 绝不能接 11.1 V**。

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

### 2.4 STM32 引脚分配（已核对无冲突）

| 引脚 | 功能 | CubeMX 设置 | 状态 |
|---|---|---|---|
| **PA8** | 电机 PWM | TIM1_CH1，AF Push Pull，Very High，Label `MOTOR_PWM` | ⬜ |
| **PA4 / PA5** | 方向 | GPIO_Output，PP，No pull，**Low**，Label `AIN1` / `AIN2` | ⬜ |
| **PA6 / PA7** | 编码器 A/B | TIM3_CH1/CH2，Encoder Mode，Pull-up，Label `ENC_A` / `ENC_B` | ⬜ |
| **PA9 / PA10** | USART1 TX/RX | Asynchronous，115200 8N1，AF PP / Input | ⬜ |
| **PB6 / PB7** | OLED SCL/SDA | GPIO_Output，**Open Drain**，**High**，Label `OLED_SCL` / `OLED_SDA` | ⬜ |
| **PC13** | 板载 LED | GPIO_Output，PP，**High = 灭**（低电平点亮），Label `LED` | ⬜ |
| **PA13 / PA14** | SWD | SYS → Debug = **Serial Wire** | ✅ |
| **PD0 / PD1** | 8 MHz 晶振 | RCC → HSE = Crystal/Ceramic Resonator | ✅ |

冲突已避开：TIM1 只用 CH1（PA8），所以 PA9 让给 USART1；编码器用 TIM3 不用 TIM1；
PB6/PB7 只当普通 GPIO（软件 I2C），TIM4 不使用。

---

## 3. CubeMX 配置（`Pid.ioc`）

| 外设 | 模式 | 参数 | 状态 |
|---|---|---|---|
| 时钟 | HSE 8 MHz → PLL ×9 → **72 MHz** | AHB /1，APB1 /2（36 MHz），APB2 /1（72 MHz） | ✅ |
| SYS | Debug = **Serial Wire** | ⚠️ 选 Disable 会把芯片锁死（只能改 BOOT0 救砖） | ✅ |
| TIM1 | Channel1 = PWM Generation CH1 | PSC = **0**，ARR = **3599** → **20 kHz**；Pulse = 0 | ⬜ |
| TIM3 | Combined Channels = **Encoder Mode** | TI1 and TI2（4 倍频），IC1/IC2 Filter = **6**，Polarity Rising，PSC = 0 | ⬜ |
| TIM2 | 时基，不占引脚 | PSC = **7199**，ARR = **49** → **5 ms**；勾 Update interrupt | ⬜ |
| NVIC | TIM2 抢占优先级 = **1** | SysTick 保持 15；USART1 中断先不开 | ⬜ |
| USART1 | Asynchronous | 115200 / 8 / None / 1，无流控 | ⬜ |
| GPIO | 见 §2.4 | PA4/PA5、PB6/PB7、PC13 | ⬜ |
| 工程 | Toolchain = **CMake**，Name = `Pid` | 位置 `/home/niko/stm32/PID/Pid` | ✅ |

注释写着 `USER CODE BEGIN/END` 的区块以外不要手改 —— 重新生成会被覆盖。
**工程名别改**：`Pid.elf` 被 `.vscode/launch.json` 和烧录脚本引用，改名要三处一起改。

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

**铁律**：`pid.c`、`vofa.c` 不许 include 任何 HAL 头 → 正确性在 `/tmp` 里就能验证，不等硬件。
新增 `.c` 必须登记 `CMakeLists.txt` 的 `target_sources`，否则编译过、链接报 `undefined reference`。

### 进度

- [x] CubeMX 骨架 + 72 MHz + SWD，`Pid.elf` 能编出来
- [ ] 外设配置（TIM1 / TIM2 / TIM3 / USART1 / GPIO）
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
