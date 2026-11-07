# Motion31 支撑中心接口与PD执行

分支：`feature/rl-motion31-supported015`。本分支只保留
`motion_multiheight_support_center_poly5_015_80_v1`，腿中心偏移范围±0.15rad、轮目标范围±80rad/s，
没有旧接口开关或兼容解码。
构建时不再传 `RL_MOTION31_OUTPUT_VERSION`。

## 模型与通道

输入31D，输出6D；四腿的真实速度进入Actor。六通道固定为模型left hip/active/wheel、
right hip/active/wheel，对应几何右三通道在前、几何左三通道在后。几何左右以站车尾朝车头看为准。
关节标定零位与正方向没有改变，不能按几何左优先重排数组。

当前`tjb_network`已通过`tools/update_xcube_ai_weights.py`更新为C0小模型
（31→128→64→32→6，14,630参数，FP32权重58,520字节），已用完整V4延迟执行器补教。
来源PT/ONNX、冻结高度中心契约、432组真实状态golden与SHA记录保存在
[当前模型目录](models/motion_C0_height_formula_trial/firmware_binding.json)。旧模型目录保留。
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

运行中允许更改命令高度，采用原合同0.30–0.40m定义域。以0.34m为例，C0四腿输出中心（ch0/1/3/4）：

```text
[-0.13097405434, -0.15117678046, +0.13097405434, -0.15117678046] rad
腿绝对目标 = 对应中心 + 0.15 * clip(action, -1, 1)
轮速目标   = 80 * clip(action, -1, 1) rad/s
```

中心由解析几何公式和全局五次多项式偏移计算，不是编码器零位或额外力矩补偿。
公式系数、中心C参考及权重必须使用同一包，不能单独替换ONNX。
编码器delta仍UNKNOWN，现有硬件标定不改，不从日志均值或中心差猜填。
不累加、不再Tanh、不按实测高度重新移动中心。解码数学范围为命令高度0.30–0.40m，
超过源定义域不外推；不新增0.32–0.36m限制。只有0.32/0.34/0.36m有旧中心的站立锚点证据，
动态高度切换与新连续公式尚未做物理验证。零位与输出中心独立，不暗加trim或增量动作。
`ChassisSetRLHeight(h)`可直接设置命令高度，原`changeL=-1/+1`拨杆升/降沿用原每周期`.002*.03m`增量。
没有停车条件、站稳计时或新增升降速度上限，vx/yaw指令不强制清零。
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
本次已接入C0固件权重和连续高度中心；没有刷写设备或修改训练进程。

## 本机验证

```sh
sh tests/run_rl_host_tests.sh
CC=clang sh tests/run_rl_host_tests.sh
# 原新接口测试入口仍调用同一套测试
sh tests/run_rl_supported015_tests.sh
```

覆盖015/80尺度/连续高度公式、旧40尺度预览拒绝、真实dq、输入归一化、关节/轮方向、真实RLShadowTask、
实际采用previous、5ms新反馈PD、P/D各通道、限矩、六轴失败事务、过期/非有限输入和故障锁存。
±80rad/s目标采用后，额外核验轮Kv0.2、previous÷100与原电流限幅，扩大目标包络不提高力矩/电流上限。
另有400组Actor/PD相位、推理耗时、任务先后顺序和时钟回绕测试。
仅RTOS/AI库等硬件边界使用桩；通过主机测试不等于目标板完整构建、实时性或实机稳定性已验证。
当前主机未提供ARM GCC，因此未做整板链接/刷写。

## 当前网络更新与核对

```bash
python tools/update_xcube_ai_weights.py .Doc/models/motion2320_actor20/actor.onnx --target tjb_network --dry-run
python tools/update_xcube_ai_weights.py .Doc/models/motion2320_actor20/actor.onnx --target tjb_network
python tools/test_tjb_network_current_weights.py
```

更新器仅替换`tjb_network_data_params.c`的权重数组，不修改其他网络、拓扑或接口。
本次已核对固件数组与ONNX逐字节相同；数组独立前向的256组golden最大误差
低于`1e-5`；真实`RLMotion31DecodeOutput`的256组输出与冻结015/80契约一致。
旧ST自动生成报告及图元数据仍记录最初生成的模型，当前权重身份以模型目录的manifest为准。

本次新增6,000次监督更新，修正了上一版四腿q/dq未进20ms Actor报文队列的问题。
现在Actor前16维全部物理观测统一标称20ms，包括四腿q/dq；previous和命令不随物理报文延迟。
腿执行保留完整V4响应。几何右轮ch2为7ms、左轮ch5为11ms的post-PD等效运输，tau0；
明确替换旧轮20ms+tau2，不重复叠加。每个最终回合均核对真实报文来源和轮力矩到达时序。
时序记录在[execution_timing.json](models/motion2320_actor20/execution_timing.json)，ONNX元数据也包含同一声明。
旧提交dff9a0b的统计不证明全物理20ms条件，旧结果只作为历史证据保留。

当前仍部署单个14,630参数MLP。34个新时序工况中，教师和学生各完成30项，学生无新增跌倒；
教师完成的30项中26项满足全部相对保真门限，核心18项全部通过。
剩余4项涉及+2m/s直行速度/轮目标抖动、combined-2m/s腿速度与俯仰峰、combined-4rad/s侧倾峰；
具体教师/学生数值在当前模型manifest.remaining_fidelity_gaps中，不宣称完全复刻。
3项无延迟基础对照也通过。完整两组延迟参数和原始结果见当前模型manifest指向的部署包。
本次只更新权重，不在固件中新增时延、滤波、补偿或提高限矩。
该结果不是目标板ST runtime、实时性或实机验证。

## C0试验范围与记录

C0是站立／直线试验候选：两个初态seed站立8/8、交接2/2、受扰恢复8/8，±2m/s直线4/4。
转向与+3m/s最差5秒支撑未过，站立仍约-0.199m/s后漂，不称完整运动或实机已通过。
先用高度0.34m、vx/yaw=0核对。现有动作保护、硬件方向、零位、PD40/2/.2、实际采用previous与5ms控制owner保留。
Actor物理观测20ms/目标5ms/完整V4/轮几何右7ms左11ms是仿真验证配置；实机用最新反馈，
不人为增加等待、不重复叠加电机延迟。

源PT SHA `8a2e5f89f76247eb5a0c4b03b03ab45833061610a355d03eada5ed3e601413ac`；
源合同 SHA `1db5cb51a7d102e738a6ddeb65406f8108463f6eac3dca1be430f9587e4734e5`。
按模型包`synchronized_capture_fields.json`记录输入、mean、中心、请求、实际采用、PD与力矩及对应时刻。
包内manifest/README描述导出时状态，当前源码集成状态单列firmware_binding.json。旧包保留；新PPO大Actor不等于本电控小模型。

## 连续高度公式与异步采用

当前有效包为[连续公式包](models/motion_C0_height_formula_trial/README.md)，原C0查表包与源PT保留。
全局五次多项式通过六个原锚点；运行时只做Horner多项式和解析几何计算，不查表、不分段插值。
与旧表插值在锚点间的最大差为髋.003870rad、主动.001535rad，Kp40对应约.155/.061Nm请求变化；
这是新解码版本，不能按旧公式物理能力全部自动通过解释。

Actor输入和解码中心使用同一包内obs[24]的命令高度。当前高度命令可在目标运输期间改变；
在途包仍使用当时高度计算出的绝对物理目标，不能按最新高度重解码。
采用层仍检查包内action与目标的解码一致性、因果previous序号、epoch、新鲜度。
新的命令高度本身不改写已采用目标或previous；采用后才更新绝对目标缓存。
PD不重加/重扣中心，不混用编码器零位。详细数值及CPU证据见[C0接口检查](C0_INTERFACE_AUDIT.md)。
