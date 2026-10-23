# 方案 1 验证记录

2026-10-23 已连接 STM32F103 与 OLED，烧录方案 1 Debug 并完成静止波形连续性、OLED 单块耗时实测。运动验收首次 S80 触发 NO_PULSE 后已自动停止；尚未获得有效的响应、静差或扰动恢复成绩。

## 已通过

- `bash tests/run_host_tests.sh`：五组主机测试，以 GCC C11、`-Wall -Wextra -Werror -O2` 编译。PID 20 项、命令 60 项、VOFA 11 项；速度环和 UI 测试包含逐 tick 的采样周期、输出和传输预算断言，不将每拍断言计作独立功能用例。
- 速度环：半截命令急停、停止锁存与显式恢复、同 tick 请求顺序、正反向无脉冲/堵转/方向异常当拍输出零、换向 300 ms 过渡、延迟清故障、相对启用时刻自动阶跃、标定进出、临界区恢复原 PRIMASK。
- UI：格式化不写总线、每 tick 至多一次且不超过 4 字节、相同内容不重写、缩短内容清尾、故障/标定切换、发送中合并最新内容、初始化失败及运行 NACK 后停止刷新。
- VOFA：原 16 字节三通道帧不变、同 tick 不重复、漏 tick 计数但不补发、关闭期间不累计、tick 回绕。
- `python3 ../tools/serial_probe.py selftest`：7 组合成波形通过；现有自动调参工具的状态解析器接受新增 `miss=` 字段。
- Debug 和 Release 完整编译链接，无编译警告；`git diff --check` 通过。
- CubeMX 文件在 USER CODE 块外的内容未改变；原有 CMake、编码器三点滑动平均改动逐字节保留；PID 实现及增益、死区配置不变。

## 本机构建产物

| 版本 | FLASH | RAM | 路径 |
|---|---:|---:|---|
| 优化前基线（含原有本地改动） | 33,392 B | 3,592 B | `build/plan1/baseline/Pid.elf` |
| 方案 1 Debug | 35,724 B | 3,736 B | `build/plan1/Debug/Pid.elf` |
| 方案 1 Release | 24,656 B | 3,744 B | `build/plan1/Release/Pid.elf` |

各目录均有 `Pid.bin`，校验值见 `build/plan1/SHA256SUMS`；这些为忽略的本机产物。基线源码暂存位置及对应 HEAD 记录在 `baseline/SOURCE.txt`。测试成功不能证明对应固件已烧入板子。

复现构建：

```sh
cmake -S . -B build/plan1/Debug -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
cmake --build build/plan1/Debug
```

将两个 `Debug` 都换为 `Release` 可复现优化构建。

## 已完成真机验收（2026-10-23）

同一板子、同样三点编码器平均与增益，对优化前重建基线和方案 1 Debug 各采集 60 秒。测试期间目标和输出均为零，OLED 已应答。

| 指标 | 优化前基线 | 方案 1 |
|---|---:|---:|
| 平均波形帧率 | 165.05 帧/s | 200.03 帧/s |
| 最大接收间隙 | 158.74 ms | 6.18 ms |
| 接收间隔 P99 | 16.73 ms | 5.23 ms |
| 超过 30 ms 的间隙 | 60 次 | 0 次 |
| 新版固件窗口末尾漏发计数 | 不支持 | `miss=0` |

帧率提高约 **21.2%**，最大接收间隙缩小约 **96.1%**。这是遥测连续性的收益，不表示电机响应加快。时间来自现有 SerialPlant 的接收块插值，不是 MCU 时间戳；结论限于本次静止采集窗口。采集结束后的显式 `?` 查询造成 `miss` 后续增加 1，属于查询阻塞，不计入 60 秒窗口。

OpenOCD 烧录显示 `Verified OK`。读取 `s_have_screen=1`，OLED 四字节写入返回 ACK；用硬件断点与 DWT 计数器测得一次写块 **75,785 cycles，约 1.053 ms @72 MHz**，小于 2 ms 预算。已确认调试暂停时计数器不增长。该测量是一个样本，不能代表全部运行条件下的最坏耗时，也不能代替人工确认屏上文字。

基线首次 S80 测试采到 497 帧，实际转速始终为零，输出曾到 1000‰，随后触发 `f=2`（NO_PULSE）。脚本自动发送 X；最终 `tgt=0 out=0 f=0`。供电开关、驱动使能或反馈链路尚需现场确认，因此未继续运动实验，未以无效数据计算改善幅度。

原始波形（gzip CSV）、失败阶跃和统计 JSON 已保存到 `tests/acceptance/2026-10-23/`。当前板上原 Flash 的完整备份位于本机 `build/plan1/acceptance/original_flash.bin`；它与用于同条件对比的重建基线不同。最终使用方案 1 固件。

## 待完成的运动及现场验收

1. 静止连续性已通过；待电机供电/反馈确认后，在运行状态下再采集 60 秒复测连续性。
2. OLED 单块 DWT 耗时抽样已通过；仍需用逻辑分析仪验证 `X/x` 接收至输出撤销不超过 10 ms，覆盖数字中、错误行和超长行。
3. 验证正反向保护、正常换向、自动阶跃时序、标定进出保持停止、缺屏和断屏行为。OLED 仍有初始化清屏和显式 `?` 文本发送等阻塞操作，不能把它们当作稳定采样窗口。
4. 同电池、负载与增益，80/150/200 rpm 各重复三次阶跃，记录响应、超调、静差；复测短时扰动恢复。与基线比较，差异超过一次采样周期或静差恶化时先排查原因。

本轮保留 PID 原有抗积分饱和行为；前馈后最终饱和未反馈到积分器，留待单独修改和重测。

复现实测（运动前确认电机可自由转动、串口独占；工具保留增益，退出时发送 X）：

```sh
python3 tests/hardware_acceptance.py --mode continuity --secs 60 --out build/plan1/acceptance/recheck
python3 tests/hardware_acceptance.py --mode steps --i-own-the-rig --out build/plan1/acceptance/steps
python3 tests/hardware_acceptance.py --mode functional --i-own-the-rig --out build/plan1/acceptance/functional
```
