# Motion31 支撑中心接口与PD执行

分支：`feature/rl-motion31-supported015`。本分支只保留
`motion31_kinematic_support_centered_015_80_v1`，腿中心偏移范围±0.15rad、轮目标范围±80rad/s，
没有旧接口开关或兼容解码。
构建时不再传 `RL_MOTION31_OUTPUT_VERSION`。

## 模型与通道

输入31D，输出6D；四腿的真实速度进入Actor。六通道固定为模型left hip/active/wheel、
right hip/active/wheel，对应几何右三通道在前、几何左三通道在后。几何左右以站车尾朝车头看为准。
关节标定零位与正方向没有改变，不能按几何左优先重排数组。

当前`tjb_network`已通过`tools/update_xcube_ai_weights.py`更新为Motion2320蒸馏小模型
（31→128→64→32→6，14,630参数，FP32权重58,520字节）。来源ONNX、冻结输出契约、
256组golden与SHA记录保存在[模型目录](models/motion2320_small31/manifest.json)。
部署模型必须明确声明上述015/80契约；015/40或旧1.5/100模型均不能直接配用本解码器。
2048大Actor不能直接写入现有31→128→64→32→6的权重数组；需先蒸馏或重新生成匹配网络。

## 输入与输出

|索引|Actor输入|
|---|---|
|0–2|模型机体角速度rad/s ×0.25|
|3–5|同坐标系单位重力向量|
|6–9|模型腿角 `[q0,q1,q3,q4]` ÷1.5|
|10–15|六关节实际速度rad/s ×0.05，不清四腿dq|
|16–19|上一实际采用腿目标 `[ch0,ch1,ch3,ch4]` ÷1.5|
|20–21|上一实际采用轮目标 `[ch2,ch5]` ÷100|
|22–24|vx命令÷4、yaw速度命令÷4、高度命令÷0.5|
|25–30|固定 `[1,0,0,1,0,0]`，Motion/NONE|

当前训练固定0.34m。四腿输出中心（ch0/1/3/4）：

```text
[-0.02911608294, -0.09327510744, +0.02911608294, -0.09327510744] rad
腿绝对目标 = 对应中心 + 0.15 * clip(action, -1, 1)
轮速目标   = 80 * clip(action, -1, 1) rad/s
```

中心是几何角加固定支撑目标偏移，不是编码器零位或额外力矩补偿。
不累加、不再Tanh、不按实测高度重新移动中心。其他高度命令拒绝。
`RLMotion31DecodeOutput(action, command_height_m, target)`是唯一解码入口，
推理预览和采用前校验共用该函数。输入归一化仍用1.5/100，不能跟着输出改成0.15/80。
例如实际采用轮目标+80/-80rad/s，下一帧obs[20:22]为+0.8/-0.8，而非+1/-1。

## 实际采用与PD

- Actor每10ms发布目标；控制owner每5ms使用当前有效反馈重算PD，两个周期不混同。
- 腿：`tau_PD=40*(q_target-q)-2*dq`，模型关节坐标、rad和rad/s，PD矩限±40Nm。
- 轮：`tau_PD=0.2*(omega_target-dq)`，PD层限±5.5Nm，最终仍受原轮电流raw±8192限制。
  按源码换算约±2.44695Nm，未作为实测输出能力。没有提高限流或改变PD增益。
- 没有重复把输出中心当零位扣除，也不将0.15动作尺度再乘进PD增益。
- 采用后previous只来自本地owner实际采用的物理目标，缓存本身不归一化。
  影子推理不写执行缓存，不接受推理历史来源。无效previous、旧epoch、过期或非因果目标拒绝。
- 接管前仍为明确标记的 `INIT_PROXY`：腿q和轮dq；不冒称LQR历史执行目标。
- 六轴目标、PD/补偿结果全部有效后才提交采用状态。任一通道非有限、反馈超龄或补偿失败时，
  沿用已有LQR故障路径并锁存RL故障，不部分采用新目标。
- 保留原5秒伸腿到腿长0.28m及随后5秒等待。腿长0.28m与root高度命令0.34m不是同一测点。
- 原弹簧补偿、硬件零位、限矩、驱动、任务周期及真实反馈获取未在本次接口任务中重构。

修复了首个目标序号处于高半区被误拒，以及已完成等待在uint32时钟回绕后撤销ready的问题。
没有修改训练进程或固件权重，也没有刷写设备。

## 本机验证

```sh
sh tests/run_rl_host_tests.sh
CC=clang sh tests/run_rl_host_tests.sh
# 原新接口测试入口仍调用同一套测试
sh tests/run_rl_supported015_tests.sh
```

覆盖015/80尺度/固定中心、旧40尺度预览拒绝、真实dq、输入归一化、关节/轮方向、真实RLShadowTask、
实际采用previous、5ms新反馈PD、P/D各通道、限矩、六轴失败事务、过期/非有限输入和故障锁存。
±80rad/s目标采用后，额外核验轮Kv0.2、previous÷100与原电流限幅，扩大目标包络不提高力矩/电流上限。
另有400组Actor/PD相位、推理耗时、任务先后顺序和时钟回绕测试。
仅RTOS/AI库等硬件边界使用桩；通过主机测试不等于目标板完整构建、实时性或实机稳定性已验证。
当前主机未提供ARM GCC，因此未做整板链接/刷写。

## 当前网络更新与核对

```bash
python tools/update_xcube_ai_weights.py .Doc/models/motion2320_small31/actor.onnx --target tjb_network --dry-run
python tools/update_xcube_ai_weights.py .Doc/models/motion2320_small31/actor.onnx --target tjb_network
python tools/test_tjb_network_current_weights.py
```

更新器仅替换`tjb_network_data_params.c`的权重数组，不修改其他网络、拓扑或接口。
本次已核对固件数组与ONNX逐字节相同；数组独立前向的256组golden最大误差
`1.02818012e-6`；真实`RLMotion31DecodeOutput`的256组输出与冻结015/80契约一致。
旧ST自动生成报告及图元数据仍记录最初生成的模型，当前权重身份以模型目录的manifest为准。

该小模型在同条件MuJoCo留出测试中，教师完成16/19条命令，学生完成17/19条命令，
学生无新增跌倒；保留教师的零命令漂移。该结果不是目标板ST runtime、实时性或实机验证。
