# C0连续高度公式与PD检查

本轮按用户最新要求开放运行中高度命令，并用连续公式替代查表。仅做源码/CPU检查，不运行训练、物理仿真或硬件。工作基线为e8c7bff；本次变更的提交与推送身份以Git历史为准。

## 当前接口与行为

- 命令高度采用原模型0.30–0.40m定义域，不额外限制为0.32–0.36m。
- `ChassisSetRLHeight(h)`直接设置绝对root高度命令；`changeL=-1/+1`沿用原升降步进。
- 无停车条件、无站稳计时、无新增高度变化速度上限、无强制清零vx/yaw。之前草稿中的这些限制均已撤销。
- 中心只由命令高度决定，不由实测高度驱动，不是编码器零偏或在线trim。

## 连续拟合公式

令`x=(h-.35)/.05`、`L=h+.000024-.060`，`beta=acos((L²+.21²-.25²)/(2*.21*L))`。

两个全局五次偏移多项式`b_hip(x)`、`b_active(x)`的系数见活动包center_fit.json，按低次到高次排列。
模型腿中心顺序为`[beta-rad(47.1033888)+b_hip, beta-rad(47.1080837)+b_active, -(第1项), 第2项]`。
运行时只有Horner多项式和解析几何，没有高度节点数组、查表或分段插值。

六个来源锚点误差最大7.21645e-16rad；锚点间相较原查表插值，髋最大差.00387013244rad，主动最大差.00153476987rad。
这些极值由每个原区间的多项式差导数求根得到，不只靠网格估计。对应Kp40请求差最多.1548053/.0613908Nm。
权重、零位和输入不变，中心函数已改变，所以动作版本单列`motion_multiheight_support_center_poly5_015_80_v1`。

## previous与PD

同一次Actor推理的obs[24]高度和目标解码中心完全一致。高度命令变化时，已计算的在途包保留其原高度和绝对物理目标，不按最新高度重算。
已采用目标仍是绝对q/轮速；命令高度改变不会清空或重标previous。新目标实际采用之后才更新previous。
腿PD使用`40*(adopted_target-q_model)-2*dq`，轮PD使用`.2*(adopted_omega-dq)`，不再加减承重中心。
合法旧高度包在运输期间返回不是错误；采用层检查该包自身的高度/目标一致性，以及原有epoch、因果previous和新鲜度。
原审查发现的“其他高度包绕过固定.34限制”在用户开放动态高度后不再按固定高度拒绝，而改为全原定义域和每包自身解码一致性检查。

原始编码器→几何角→模型q和目标逆映射已做101高度往返；四个电机P项均恢复、D项均阻尼。
实物编码器delta仍UNKNOWN；这证明源码代数一致，不能证明物理标定准确。原硬件offset、PD增益40/2/.2、限矩、弹簧补偿与驱动内kp/kd置零路径不变。

## CPU证据

- GCC/Clang各8项控制回归通过，保留400组Actor/PD时相和时钟回绕。
- 全定义域101高度的输入/中心/PD及previous对照通过。
- 非零运动命令下直接从.34→.38→.30，延后返回的旧包仍按原中心采用，previous只来自实际采用，PD与绝对目标一致。
- 实际CurrentRLSnapshot/ChassisSetRLHeight源码抽取后在CPU硬件替身上执行，GCC/Clang均通过，保留运动指令及原始dq/零位映射。
- 5项网络/公式对照通过；432组真实反馈golden通过，固件mean误差2.75671482e-7；1001点及原锚点的中心golden通过。
- ASan/UBSan的动态高度回归通过。权重更新工具dry-run确认权重数组无需改变。转换器4项通过，另2项依赖缺失的旧25D模型按原规则跳过。

本轮未验证动态升降的闭环能力，不因CPU通过宣称不同高度运动或实机已通过。原C0转向/高速和后漂缺口保留，不添加限制替代这些事实。

## 修改入口

`application/chassis/chassis.c/.h`接入命令；`rl_motion31.c/.h`连续中心与输入域；`rl_control.c/.h`保存命令高度和实际采用高度身份，不重算previous。
`tests/test_rl_center_contract.c`、`test_rl_height_runtime.c`及`tests/run_rl_host_tests.sh`；`tools/fit_c0_height_center.py`、`test_rl_height_chassis_boundary.py`和网络回归；
`.Doc/models/motion_C0_height_formula_trial/`为独立活动包，旧包保持。结构化证据见`.Doc/audits/C0_interface/dynamic_formula_verification.json`。
