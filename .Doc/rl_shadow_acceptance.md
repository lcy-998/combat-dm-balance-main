# 强化学习影子推理验收记录

## 已完成的软件检查

- 分支：`feature/rl-input-shadow`。网络仅写 `g_rl_shadow_debug`，底盘仍由 LQR/VMC 输出。
- 本机观测构造测试：`gcc -std=c11 -Wall -Wextra -Werror -Iapplication/chassis tests/test_rl_observation.c application/chassis/rl_observation.c -lm -o build/test_rl_observation.exe`，执行结果为 `rl observation tests passed`。
- STM32 固件：`mingw32-make -j4` 编译链接成功；DTCMRAM 占用 82,912 / 131,072 字节，Flash 约 217.5 KB。FreeRTOS heap 配置为 40,960 字节；影子线程栈为 1,024 个 32 位字。
- 生成网络形状为 25 输入、6 输出。当前仓库没有与该 ONNX 对应的 `params/env.yaml`、`params/agent.yaml`、`action_contract.json`，模型专属缩放配置尚不能交叉核验。

## Ozone 查看与实机待验

添加全局变量 `g_rl_shadow_debug`。`publish_seq` 为偶数时是一份完整记录；`status=0` 表示本次输入和推理有效。`observation[0..24]` 按 I/O 契约排序；`raw_action[0..5]` 是未执行的无量纲网络输出。`lqr_torque_nm` 顺序为物理左轮、右轮、左腿、右腿；`motor_torque_nm` 顺序为 lf、lb、rf、rb；`wheel_current_cmd` 是物理左、右轮驱动命令计数。`heading_vx_m_s` 为滤波前速度，`lqr_speed_m_s` 为滤波后速度。

在可靠支撑、断开策略输出的条件下核对：静止时重力槽位约 `[0,0,-1]`；低头、右倾、左转时对应输入为正；四关节位于模型零位时 9–12 槽约零；前进时两轮 15、18 槽为负；19–24 槽始终为零。记录 `missed_periods`、`stack_free_min_words` 与 `heap_free_min_bytes`，最坏运行状态下要求影子线程剩余栈至少 256 字。尚未进行实机测量，因此本记录不代表实机验收通过。
