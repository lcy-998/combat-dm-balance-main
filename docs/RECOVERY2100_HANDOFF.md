# 31D Recovery2100 → supported015 Motion

此分支基于远端 `feature/rl-motion31-supported015` 的 `786a2c5`。015 的生成网络文件保持原样。网络都是 `31→256→128→64→6`，但动作合同不同，不可互换 decoder。

| 项目 | Recovery2100 | 015 Motion |
|---|---|---|
| 推理 | 本分支 `recovery2100/recovery_actor.c`，确定性 ELU/Tanh | 015 原有 X-CUBE-AI `tjb_network` |
| 腿目标 | 固定中心三次绝对目标，包络 ±1.5 rad | C0 五次高度中心 + `0.15×mean` |
| 轮目标 | `100×mean` rad/s | `80×mean` rad/s |
| skill 25–27 | `0,1,0` | `1,0,0` |

Recovery学生来源：`/home/kellen/wheel_leg_training_data/2026_11_13__recovery2100_mujoco_rise_r2/deployment_v3`。唯一教师2100 SHA256为 `5aec83df5644ae1b96f4ed012be41d41cb15c4729bbc7b632af347912d3d12a9`。本次没有训练新网络、修改教师或混用其他Recovery教师。

## 正式接线

`StartMOTORTASK → ChassisRLRecoveryTick → RLControlExecute → recovery_execute → 六通道本地提交 → MotorControlTask`。

只在显式 Recovery 会话走此链。普通015路径及其原有8组CPU回归保留。Recovery会话跳过旧LQR复位、摆腿、VMC与离地补偿输出，故障后六通道归零并锁存，不在Swing中隐式回退LQR。

策略任务仍10 ms。Recovery会话执行1 ms、PD5 ms、目标运输5 ms、模型两侧轮力矩运输7/11 ms。收到的31D状态包名义20 ms，启动暖填，真实输入tick记录到capture v2，新增Recovery模式与Motion候选有效位；读取器须按schema_version/record_bytes识别，不能按旧v1固定长度解释。反馈采集/IMU更新已有本地时间戳；它们不代表电机采样时刻或远端执行确认。硬件本身的电机动态不再叠加软件电机模型。

`previous` 取本地实际采用的物理目标。首次采用前明确记为测量q/零轮速初始化proxy。Motion探测只读同一收到状态及previous，不写采用缓存。交接时保留状态包、目标队列、轮运输队列和previous；更新epoch，拒绝旧epoch输出。两种decoder各执行一次。

PD为40/2/0.2，Recovery会话腿40 Nm、轮4.8 Nm。原015普通Motion合同保持。驱动原始轮电流限幅保留16384，不提高驱动限幅；Nm到原始指令转换沿用015，实际电流/力矩标定尚未实测。Recovery会话不加旧LQR弹簧补偿。

## 显式启动和交接证据

1. 保持既有 `CHASSIS_ZERO_FORCE` 安全模式。
2. 上层调用 `ChassisStartRLRecovery()`，或写入显式请求邮箱 `g_rl_recovery_request=1`，由底盘任务在安全模式消费。请求只arm，不覆盖急停或使能决定。
3. 既有外部控制将底盘切到非零力模式后，Recovery取得控制权。起身阶段命令固定为0/0/0.34；首次Motion探测使用相同命令。
4. 合格平台估计器经 `ChassisUpdateRLHandoffEvidence()` 发布带真实tick的高度、姿态、角速度、两轮独立支撑和最大腿速度。证据过期、缺失或失效时不交接。
5. 质量连续满足200 ms后请求Motion探测；新鲜候选通过物理目标跳变量检查后一次交接。之后在同一会话内由Motion控制，不来回切换。
6. 停止/故障后回安全模式清除会话。重新起身需要新显式请求。

当前质量阈值：高度0.30–0.40 m且距0.34 m不超过0.02 m；pitch/roll各≤0.10 rad；角速度范数≤1 rad/s；最大腿速度≤3 rad/s；每轮法向力>2 N。Motion候选相对实际previous的最大腿目标跳变≤0.4 rad、轮目标跳变≤40 rad/s。200 ms是短时接管准备条件，后续10秒保持由整个Recovery→Motion组合验证，不要求Recovery先保持10秒。

**实机支撑与高度生产者尚未接入。** 现有 `F_n` 使用指令力矩等估计，不能作为合格双轮接触证据；命令高度和FK腿长也不等于实测离地高度。当前缺省证据为UNKNOWN，硬件上不会自动交接。API保留在ELF中供合格平台生产者接入。仿真使用物理真值验证门槛，不把真值可用性冒充硬件能力。

## 验证与构建

运行 `sh tests/run_rl_host_tests.sh`，覆盖原015消费路径及Recovery目标、PD/运输时序、质量门、epoch与故障。完整ARM构建命令：

```sh
make -j12 SHELL=/bin/sh \
  GCC_PATH=/home/kellen/.local/opt/xpack-arm-none-eabi-gcc-14.2.1-1.1/bin \
  OPT='-Og -fstack-usage'
```

原仓Linux大小写错误已修正：SEGGER汇编文件为小写`.s`，DM头文件为`dmmotor.h`。新增权重没有覆盖015的X-CUBE-AI Motion。capture环形缓存移到RAM_D1并由`RLCaptureArm`显式初始化；shadow栈8192 B、电机任务栈4096 B、RTOS heap32768 B。静态栈报告不能代替目标MCU高水位和实时截止时间实测。

128条真实状态golden：C mean最大误差4.2934e-7，物理目标最大误差2.28882e-5。Flash/RAM精确链接结果、源码SHA和测试结果在 `artifacts/recovery2100_integration/`。本次未刷机、未驱动真实电机、未验证任意倒地方向；能力范围先限定为教师/学生已验证的合法Swing。

闭环对照经训练仓统一 `wheel-leg sim2sim` 的 `recovery_motion_handoff_v1` 执行。原始证据位于数据根 `recovery_motion_handoff/`；所有旧run和失败证据保留。综合结果见同目录 `SIMULATION_RESULTS.md`。
