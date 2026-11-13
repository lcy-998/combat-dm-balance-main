# Recovery2100起身 → 31D 015 Motion自动接管

本分支为STM32H723实机试验版本，基于015远端`7f89e6d`及Recovery2100小模型。两套31→256→128→64→6网络均在本仓，不需要从训练机另取权重。015生成网络保持原样。

## 本次默认流程

1. 上电后保持遥控器零力/急停安全模式，等待六电机与IMU反馈正常。
2. 安全模式下程序自动arm Recovery；切换到原有非零力使能模式后，由Recovery起身，跳过原LQR复位/摆腿。
3. 起身期间vx/yaw/height命令固定0/0/0.34。使用本体IMU与编码器判断准备条件，持续100 ms后只读探测Motion。
4. Motion新鲜输出相对实际previous的腿目标跳变≤0.4 rad、轮目标跳变≤40 rad/s时自动接管。过程中不reset、不清previous或运输队列，不加混合插值、滤波或弹簧补偿。
5. 接管后Motion响应原有摇杆速度、转向和腿高控制。同一会话不来回切换网络。
6. 切回零力模式立即沿原有电机停止路径退出。反馈或策略失效则锁存归零，不在Swing中回退LQR；回安全模式后才能重新开始。

配置入口：`application/chassis/rl_recovery_config.h`。

- `RL_RECOVERY_AUTO_START=1`：安全→使能每次从Recovery开始。
- `RL_RECOVERY_POSTURE_HANDOFF=1`：默认本体感知准备判据。
- `RL_RECOVERY_POSTURE_DWELL_MS=100`：本体感知满足条件后尽早交给Motion；外部严格支撑门仍独立为200ms。
- `RL_RECOVERY_POSTURE_ERROR_RAD=0.50`：四腿实际位置相对Motion0.34m参考中心，各自允许偏差0.50 rad。该中心是输出参考，不是编码器偏置；不能改编码器零位来凑中心。
- 若改`POSTURE_HANDOFF=0`，则必须由合格平台提供实际高度/双轮支撑证据，否则不会交接。

## 本体感知自动交接

使用真实六路电机CAN接收时间戳、IMU更新完成时间戳、四腿q/dq和IMU重力/角速度。反馈全六路及IMU须有效；控制新鲜度≤24 ms，准备判据年龄≤20 ms。dwell按最旧必需反馈的接收时间推进，重复同一反馈不能由1ms循环累积成200ms。

条件：单位重力合法；重力方向距直立轴≤0.10 rad；角速度范数≤1 rad/s；四腿速度绝对值各≤3 rad/s；四腿位置距Motion参考中心各≤0.50 rad。连续100ms后仍需通过新鲜Motion输出/previous/epoch/物理目标跳变检查。

这是`handoff_source=2`的姿态与构型准备条件，不是独立触地证明。悬空但姿态和构型满足时也可能接管。现有`F_n`含指令力矩，未拿它冒充双轮承重；没有给法向力填常数、没有使用外部定位或仿真高度。若平台提供有效真实高度/双轮支撑证据，使用`handoff_source=1`的原严格门；切换证据来源会清除未完成dwell。

调试观察 `g_rl_shadow_debug`：

| 字段 | 含义 |
|---|---|
| `recovery_mode` | 1=Recovery，0=Motion |
| `handoff_source` | 1=合格支撑证据；2=本体姿态/构型准备 |
| `handoff_requested` / `handoff_committed` | 请求只读Motion探测 / 已提交交接 |
| `failure` / `inference_deadline_miss_count` | 推理失败原因 / 超10ms推理次数 |
| `publish_seq` / `frame_end` | 两者相等且偶数时读取一致快照 |

capture为v3，新增source/requested/committed及实际Actor输入tick。读取器按`schema_version/record_bytes`识别，不按旧固定长度解释。

## 模型与执行合同

| 项目 | Recovery2100 | 015 Motion |
|---|---|---|
| 网络 | `recovery2100/recovery_actor.c`，确定性ELU/Tanh | 015原X-CUBE-AI `tjb_network` |
| 腿目标 | 固定中心三次绝对目标，±1.5rad包络 | C0五次高度中心+0.15×mean |
| 轮目标 | 100×mean rad/s | 80×mean rad/s |
| skill25–27 | 0,1,0 | 1,0,0 |

输入均31D，保留真实dq。输入腿/previous腿除1.5，previous轮速除100；previous只来自本地实际采用物理目标，首次采用前明记q/零轮速初始化proxy。影子推理不能写缓存，decoder各执行一次。模型left对应几何右，不交换数组。

策略10ms。显式Recovery→Motion会话由电机发送前1ms入口拥有六通道输出；PD5ms，目标运输5ms，模型双轮力矩运输7/11ms。收到状态包名义20ms、启动暖填。PD40/2/0.2；腿40Nm、轮4.8Nm；保留015驱动原始轮电流限幅16384，实际电流/力矩比例未实测。此会话不加旧LQR弹簧补偿；真实机械弹簧及驱动配置仍需按本机平台核对。普通015传统入口保留，但默认自动Recovery会话使用上述明确合同。

唯一教师2100 SHA256：`5aec83df5644ae1b96f4ed012be41d41cb15c4729bbc7b632af347912d3d12a9`。本次没有新训练或混用其他Recovery教师。源部署包：`/home/kellen/wheel_leg_training_data/2026_11_13__recovery2100_mujoco_rise_r2/deployment_v3`。

## 构建与交付

```sh
sh tests/run_rl_host_tests.sh
make -j12 SHELL=/bin/sh \
  GCC_PATH=/home/kellen/.local/opt/xpack-arm-none-eabi-gcc-14.2.1-1.1/bin \
  OPT='-Og -fstack-usage'
```

可烧写hex/bin与SHA在`firmware/recovery2100_motion015/`，bin加载地址`0x08000000`；源码构建仍生成`build/Basic_Framework_MC02.*`。本次未连接设备或刷机。栈与实时期限目前有静态构建和计数器，尚无目标MCU高水位/推理时限实测。

CPU原0158组与Recovery控制/真实shadow14情景通过；128条真实golden误差mean4.29e-7、物理目标2.29e-5。闭环本体判据结果与失败边界见`docs/SIMULATION_RESULTS.md`和release验证收据。能力范围是已测合法Swing，不是任意倒地方向的实机成功认证。
