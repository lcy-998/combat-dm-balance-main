# 强化学习影子推理验收记录

## 已完成的软件检查

- 分支：`feature/rl-input-shadow`。网络仅写 `g_rl_shadow_debug`，底盘仍由 LQR/VMC 输出。
- 本机观测构造测试：`gcc -std=c11 -Wall -Wextra -Werror -Iapplication/chassis tests/test_rl_observation.c application/chassis/rl_observation.c -lm -o build/test_rl_observation.exe`，执行结果为 `rl observation tests passed`。
- 网络输出换算测试：`gcc -std=c11 -Wall -Wextra -Werror -Iapplication/chassis tests/test_rl_output_compare.c application/chassis/rl_output_compare.c -lm -o build/test_rl_output_compare.exe`，执行结果为 `rl output comparison tests passed`；覆盖用户提供的全部查表点、插值、速度绝对值及上限、动作裁剪和髋角左右符号。
- STM32 固件：`mingw32-make -j4` 编译链接成功；DTCMRAM 占用 82,960 / 131,072 字节，Flash 约 218.4 KB。FreeRTOS heap 配置为 40,960 字节；影子线程栈为 1,024 个 32 位字。
- 生成网络形状为 25 输入、6 输出。当前仓库没有与该 ONNX 对应的 `params/env.yaml`、`params/agent.yaml`、`action_contract.json`，模型专属缩放配置尚不能交叉核验。

## Ozone 查看与实机待验

添加全局变量 `g_rl_shadow_debug`。`publish_seq` 为偶数时是一份完整记录；`status=0` 表示本次输入和推理有效。`observation[0..24]` 按 I/O 契约排序；`raw_action[0..5]` 是未执行的无量纲网络输出。`lqr_torque_nm` 顺序为物理左轮、右轮、左腿、右腿；`motor_torque_nm` 顺序为 lf、lb、rf、rb；`wheel_current_cmd` 是物理左、右轮驱动命令计数。`heading_vx_m_s` 为滤波前速度，`lqr_speed_m_s` 为滤波后速度。

输出对照数组均按物理左、右排序：`policy_wheel_torque_nm` 是正值表示前进的网络轮端力矩预览，`lqr_wheel_torque_nm` 为现有轮力矩；`policy_hip_q_target_rad` 为模型髋关节目标角，`policy_hip_target_angle_rad` 为换算到 `lbangle/rbangle` 坐标的角度，`measured_hip_angle_rad` 为实测 `lbangle/rbangle`。`leg_scale_rad_per_action` 来自用户提供的速度查表。网络动作仅作假设执行的输出换算，输入 19–24 的实际执行动作历史仍为零；本换算不验证编码器物理零位。

在可靠支撑、断开策略输出的条件下核对：静止时重力槽位约 `[0,0,-1]`；低头、右倾、左转时对应输入为正；四关节位于模型零位时 9–12 槽约零；前进时两轮 15、18 槽为负；19–24 槽始终为零。记录 `missed_periods`、`stack_free_min_words` 与 `heap_free_min_bytes`，最坏运行状态下要求影子线程剩余栈至少 256 字。尚未进行实机测量，因此本记录不代表实机验收通过。
