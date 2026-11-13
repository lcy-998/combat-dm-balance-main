# 015 Motion交接闭环结果

来源：`/home/kellen/wheel_leg_training_data/recovery_motion_handoff/firmware_motion015_swing7_r4`，逐case报告在相邻的 `firmware_motion015_swing7_r4_evidence/REPORT.md`。通过正式 `wheel-leg sim2sim` 执行；21个完整对照回合，实际471540个0.5 ms物理子步，263.83 s。Recovery与Motion在同一物理世界交接，不reset，不清previous或执行队列。无训练、无硬件输出。

| 方案 | 10秒保持通过 | 成功case交接时刻 | 首次腿目标最大跳变 | 首次轮目标最大跳变 |
|---|---:|---|---:|---:|
| Recovery单独控制 | 6/7 | 不交接 | 不适用 | 不适用 |
| 质量持续100 ms后交接 | 6/7 | 0.51–0.85 s | 0.141401 rad | 6.06742 rad/s |
| 质量持续200 ms后交接 | 6/7 | 0.61–0.95 s | 0.061325 rad | 4.75712 rad/s |

优先使用200 ms。成功的6组交接后连续保持12.05–12.39 s。这里的保持不要求零速度；最后5秒绝对vx均值0.126–0.301 m/s。HP10指标保留在原始episodes中，不声称消除振荡。200 ms方案的末5秒四腿HP10 RMS最大0.002720 rad/s，这是本组参考接触/时序下的结果。

共同失败case `development_04`：seed914005、DR0.5、yaw+90°，三种方案在0.59 s、尚未交接时故障，trace SHA完全相同。触发直立投影<0.5的原终止门；物理失稳根因待确认。此组属于当前Recovery闭环边界，不修改门槛、不计成Motion接管故障，也不声称本轮必须超越教师。

覆盖为现有7个development合法Swing条件，含yaw0/±90°、DR0/.25/.5/1，不是最终留出测试或完整DR笛卡尔积。门槛的高度/双轮支撑来自仿真真值，角速度来自有年龄的策略观测；MCU端仍需合格平台生产者。单靠定时器1秒切换虽在最初单组通过，不作为默认方案。

仿真模块：相关21项CPU测试、portable2011 passed/31 skipped、架构与Ruff0.8.4通过，详细收据保留在上述evidence目录。电控侧的CPU、真实golden、ARM完整构建分别验证接口；仿真通过不等于硬件已标定或实时deadline已通过。
