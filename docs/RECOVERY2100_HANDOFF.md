> **当前网络替换试验版：** Recovery已改为旧`tjb_recover`（c94bb9f，31→128→64→32→6），腿`1.5×mean`、轮`100×mean`。015 Motion和当前控制配置不变。仿真7组全部交接前失败，不是此前2100成功版；先前2100版可用`ddb5045`。详见[替换报告](../artifacts/tjb_recover_replacement/REPORT.md)。历史路径名不代表当前网络身份。

# Recovery2100起身 → 31D 015 Motion自动接管

本分支为STM32H723实机试验版本。当前Recovery为旧tjb_recover的31→128→64→32→6独立C网络；015 Motion为31→256→128→64→6的X-CUBE-AI网络，两套权重均在仓库内。此前2100蒸馏小模型为31→256→128→64→6，成功版回退点ddb5045；当前没有装载它。

## 仿真与实机补偿口径

“软件层面不需要补偿”不准确：电控补偿本身就是软件。应按执行路径区分：

| 路径 | 当前做法 | 证据与边界 |
|---|---|---|
| 完整V4训练/评估/sim2sim | 不额外调用软件弹簧补偿 | 保留原V4+NET；“不额外加”不表示已证明V4不含补偿作用 |
| 015普通Motion实机 | 保留原RLControlCompensateSpring | 不修改feature/rl-motion31-supported015参考分支 |
| 本集成会话实机Motion | 与Recovery共用同一015补偿 | 5ms更新/保持，最终腿限矩40Nm；交接不重复叠加 |
| 本集成会话实机Recovery | 已按用户部署方案启用同一015补偿 | 与Motion共用输出路径；实际硬件效果仍待验证 |
| 旧tjb原生c94bb9f实机 | 原生代码在Recovery输出时加补偿 | 不能把它与本集成代码现状或V4仿真配置混为一谈 |

若V4确实拟合了“软件补偿＋电机＋真实弹簧”的整体响应，则仿真不额外加补偿、实机保留对应补偿，两者可以一致。但当前缺少V4采集时补偿开关的确证，不能宣称所有模型/高度/力矩工况已认证等效。完整V4仿真成功不能证明实机Recovery应关闭补偿；当前已按用户明确部署方案让Recovery与Motion共用原015补偿，但这不等于V4采集假设已获证实。

验证完整电控（含补偿）需要独立物理弹簧、相应电机响应并重新核对V4/NET残差，不能直接将软件补偿叠加到原整套V4后当作实机验证。本轮落实的是实机软件部署方案，未修改V4仿真路径。

## 本次默认流程

1. 上电后保持遥控器零力/急停安全模式，等待六电机与IMU反馈正常。
2. 安全模式下程序自动arm Recovery；切换到原有非零力使能模式后，由Recovery起身，跳过原LQR复位/摆腿。
3. 起身期间vx/yaw/height命令固定0/0/0.34。使用本体IMU与编码器判断准备条件，持续20 ms后只读探测Motion。
4. Motion新鲜输出相对实际previous的腿目标跳变≤0.4 rad、轮目标跳变≤40 rad/s时自动接管。过程中不reset、不清previous或运输队列，不加混合插值或滤波。Recovery与Motion两阶段均复用普通015原有弹簧补偿；交接不切换补偿开关、不额外叠加补偿。
5. 接管后Motion响应原有摇杆速度、转向和腿高控制。同一会话不来回切换网络。
6. 切回零力模式立即沿原有电机停止路径退出。反馈或策略失效则锁存归零，不在Swing中回退LQR；回安全模式后才能重新开始。

配置入口：`application/chassis/rl_recovery_config.h`。

- `RL_RECOVERY_AUTO_START=1`：安全→使能每次从Recovery开始。
- `RL_RECOVERY_POSTURE_HANDOFF=1`：默认本体感知准备判据。
- `RL_RECOVERY_POSTURE_DWELL_MS=20`：本体感知满足条件后尽早交给Motion；外部严格支撑门仍独立为200ms。
- `RL_RECOVERY_POSTURE_TILT_RAD=0.35`（约20°）、`GYRO_RAD_S=3`、`LEG_SPEED_RAD_S=8`：扩大Motion接管范围。
- `RL_RECOVERY_POSTURE_ERROR_RAD=0.50`：四腿实际位置相对Motion0.34m参考中心，各自允许偏差0.50 rad。该中心是输出参考，不是编码器偏置；不能改编码器零位来凑中心。
- 若改`POSTURE_HANDOFF=0`，则必须由合格平台提供实际高度/双轮支撑证据，否则不会交接。

## 本体感知自动交接

使用真实六路电机CAN接收时间戳、IMU更新完成时间戳、四腿q/dq和IMU重力/角速度。反馈全六路及IMU须有效；控制新鲜度≤24 ms，准备判据年龄≤20 ms。dwell按最旧必需反馈的接收时间推进，重复同一反馈不能由1ms循环累积成200ms。

条件：单位重力合法；重力方向距直立轴≤0.35 rad；角速度范数≤3 rad/s；四腿速度绝对值各≤8 rad/s；四腿位置距Motion参考中心各≤0.50 rad。连续20ms后仍需通过新鲜Motion输出/previous/epoch/物理目标跳变检查。

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

| 项目 | 当前旧tjb Recovery | 015 Motion |
|---|---|---|
| 网络 | `recovery2100/recovery_actor.c`，确定性ELU/Tanh | 015原X-CUBE-AI `tjb_network` |
| 腿目标 | 1.5×mean绝对目标；仅历史2100使用固定中心三次公式 | C0五次高度中心+0.15×mean |
| 轮目标 | 100×mean rad/s | 80×mean rad/s |
| skill25–27 | 0,1,0 | 1,0,0 |

输入均31D，保留真实dq。输入腿/previous腿除1.5，previous轮速除100；previous只来自本地实际采用物理目标，首次采用前明记q/零轮速初始化proxy。影子推理不能写缓存，decoder各执行一次。模型left对应几何右，不交换数组。

策略10ms。显式Recovery→Motion会话由电机发送前1ms入口拥有六通道输出；PD5ms，目标运输5ms，模型双轮力矩运输7/11ms。收到状态包名义20ms、启动暖填。PD40/2/0.2；腿40Nm、轮4.8Nm；保留015驱动原始轮电流限幅16384，实际电流/力矩比例未实测。此会话Recovery与Motion阶段均复用RLControlCompensateSpring，按PD5ms读取实际腿长/夹角更新并保持，最终腿力矩限幅±40Nm。两阶段共用同一输出路径，删除已无用途的仅Motion补偿模式标记；交接不重复叠加补偿。真实机械弹簧及驱动配置仍需按本机平台核对。普通015传统入口保留，但默认自动Recovery会话使用上述明确合同。

历史2100蒸馏的唯一教师SHA256：`5aec83df5644ae1b96f4ed012be41d41cb15c4729bbc7b632af347912d3d12a9`。当前旧tjb替换不是该教师的蒸馏模型；没有新训练。历史2100源部署包：`/home/kellen/wheel_leg_training_data/2026_11_13__recovery2100_mujoco_rise_r2/deployment_v3`。

## 构建与交付

```sh
sh tests/run_rl_host_tests.sh
make -j12 SHELL=/bin/sh \
  GCC_PATH=/home/kellen/.local/opt/xpack-arm-none-eabi-gcc-14.2.1-1.1/bin \
  OPT='-Og -fstack-usage'
```

可烧写hex/bin与SHA在`firmware/recovery2100_motion015/`，bin加载地址`0x08000000`；源码构建仍生成`build/Basic_Framework_MC02.*`。本次未连接设备或刷机。栈与实时期限目前有静态构建和计数器，尚无目标MCU高水位/推理时限实测。

CPU原0158组与Recovery控制/真实shadow14情景通过；128条真实golden误差mean4.29e-7、物理目标2.29e-5。闭环本体判据结果与失败边界见`docs/SIMULATION_RESULTS.md`和release验证收据。能力范围是已测合法Swing，不是任意倒地方向的实机成功认证。

## 实车前兼容审查更新

两套权重均为C，Recovery是独立const float推理核，Motion为ST打包ai_u64推理，格式不同但不依赖ONNX运行。ST格式的`tjb_recover*`未参与当前活动构建，但其旧128网络权重已经提取到独立C核并作为当前Recovery；recovery2100目录名不代表当前权重身份。

本次修复模式冲突误释放会话、新会话复用旧FIFO和采集过早冻结三项bug，旧ffed642固件/ZIP被当前产物取代。15种真实shadow情景与9/9测试通过。新固件只能称可进入有保护的分步实车验证，尚无目标deadline/栈高水位及本车零位、方向、力矩标定实测证据。详细根因/定位/处理和边界见`artifacts/recovery2100_vehicle_audit/REPORT.md`。

Recovery会话采集按200Hz，在接管/故障触发并保存约555ms后续；冻结后保留供读取，不随安全模式自动清除。需要新一轮采集时显式调用`RLCaptureArm(RL_CAPTURE_CAPACITY-17u)`。

## 历史Motion补偿修复与V4复测（40e1b8b，2026-11-15）

仅修改本集成分支；feature/rl-motion31-supported015作为只读参考，不提交到该分支。当前Recovery仍是旧tjb_recover，腿1.5×mean、轮100×mean；未换回2100。

实机Motion链（含普通015及集成接管后）：PD→015补偿→最终限矩→真实驱动器/弹簧。该历史版本集成实机Recovery未调用补偿；本轮已按用户部署方案启用，见下节。仿真链：原PD→完整V4+NET；不额外叠加015补偿。V4采集是否包含当时软件补偿尚缺开关记录，因此本次不是包含独立真实弹簧的完整固件物理验证。

正式sim2sim复测：tjb_v4_recheck_motion_spring_fix_r1包含Recovery-only与20ms本体交接各7组，均0/7，全部0.49–1.21秒交接前失败；motion015_v4_recheck_motion_spring_fix_r1站立两组均通过，连续保持12.984/12.982秒。数据根：/home/kellen/wheel_leg_training_data/recovery_motion_handoff。补偿的软件行为由真实C主机消费者验证，不能将V4数值结果称为补偿实机认证。

## 实机两阶段统一补偿（2026-11-15）

用户明确要求落实实机Recovery和Motion都保留015弹簧补偿，V4仿真不额外叠加。本轮将recovery_execute的PD输出改为两阶段共用RLControlCompensateSpring，删除仅Motion启用的运输模式标记。每5ms计算：原PD限幅→映射motor顺序→加一次补偿→最终腿±40Nm；期间保持输出。零输入调用只记录spring诊断，最终发送的是对原PD输出调用的结果，不相加两份函数结果。轮速PD、轮4.8Nm限矩、网络解码、目标运输、previous及交接门不改。

Recovery尚未产生有效策略请求时沿原SAFE路径保持零输出；反馈/几何失效锁零。交接时补偿不切换、不重复、不清缓存。实际C消费者核对Recovery补偿公式/5ms保持与刷新、两阶段最终限矩、交接帧无额外项、无效几何锁零，原时序/previous/故障测试保持通过。

当前仍装载旧tjb_recover，不自动换回2100；先前V4下旧tjb0/7、2100小模型6/7是原仿真记录，不是本次实机补偿效果。仿真路径未修改，因此没有以额外叠加补偿重新做V4试验。未刷机，未驱动真实电机。
