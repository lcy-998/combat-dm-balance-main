# 旧tjb_recover替换比较

按用户要求，仅替换Recovery网络及其必须匹配的输出合同。015 Motion权重、PD、限矩、时序、previous、交接20ms均不改。活动Recovery换成远端c94bb9f的tjb_recover，31→128→64→32→6，ELU/Tanh，14630参数、58520字节。由源提交ST ai_u64原数组逐FP32位提取，生成独立C核；128真实状态对照mean误差1.78814e-7、物理目标误差1.87159e-5。来源SHA见identity.json。

腿输出必须1.5×mean，轮100×mean，不能继续使用2100三次公式。C接口/目录recovery2100为保留调用链路径，不代表当前仍装载2100。当前网络以本报告与manifest为准。旧2100成功版本回退点为ddb5045。

当前控制合同：腿PD40/2、轮速增益.2、腿40/轮4.8Nm、完整名义V4与20/5/7/11ms时序、无额外弹簧补偿。不是旧网络原60/2、轮增益1、轮5.5Nm的旧配置。没有训练。

仿真：7个development Swing条件，各跑Recovery_only与Recovery→Motion，合计14回合。两方案各0/7保持通过，所有回合在0.49–1.21秒失稳，未触发Motion交接。无补偿且改用当前PD的组合目前不具备原2100的起身效果，不能把本产物称为可靠起身版。详细逐case/实际消耗见episodes与terminal。没有证明失败单独由PD、补偿、模型来源或接口分布哪一项引起。

弹簧核对：015普通Motion的RLControlExecute非session分支调用RLControlCompensateSpring；Recovery→Motion session走recovery_execute，不调用该补偿，两阶段都无额外软件弹簧项。完整V4仍是仿真原响应合同，不能当作实机已校准。此次不加补偿，保持单变量网络比较；不是宣称015从来没有弹簧补偿。

验证：电控9/9回归、真实shadow15情景通过；ARM完整链接通过Flash499580B；仿真消费者15项通过、Ruff/架构通过。编译/数值一致不等于物理能力通过。未刷机，未实际驱动电机。现有firmware/recovery2100_motion015文件名沿用路径，但当前产物已换为本旧模型比较版，见README和manifest警示。
