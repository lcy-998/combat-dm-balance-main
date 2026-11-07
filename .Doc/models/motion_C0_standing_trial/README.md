# C0站立／直线电控试验候选

这是已冻结C0的独立电脑端试验包，未安装固件、未刷板、未实机验证。student.pt保持原SHA8a2e5f89…1413ac，不能只换ONNX沿用旧中心。

先以0.34m、vx=0、yaw=0固定命令高度核对，再按既有实机流程安排试验。本包不自行上电或驱动电机。0.32/.34/.36m仿真站立、连续教师前控→C0和小幅关节受扰恢复已通过两个初态seed；前后2m/s直线本轮通过，转向和+3m/s最差5秒承重未过。站立仍有约-0.199m/s后漂，.36m承重余量小。不能授完整运动或全DR/实机通过，受扰恢复不是Swing起身。

完整模型组合：31D→128→64→32→6，ELU/Tanh；腿q_target=c(height)+.15*mean，轮omega_target=80*mean。c必须使用本包support_center.c/h、support_center.json、output_contract.json的冻结多高度中心，Python/CPU公式与verification/golden一致。使用模型left hip/active/wheel、right hip/active/wheel数组顺序；模型left对应几何右，不交换通道。

输入保持绝对腿q/1.5、实际采用previous腿目标/1.5、previous轮速目标/100、真实dq*.05；不减承重中心，不增加观测。previous只写实际采用层，不能写影子网络输出。新mean0不是零力矩，不能通过改编码器零位追mean0。

PD40/2/.2、限矩腿40Nm/轮2.44694786377709Nm、1/5/10ms、完整V4腿响应不变；仿真验证Actor首16物理字段20ms，目标5ms，几何右/ch2轮7ms、左/ch5轮11ms且tau0。实机使用最新反馈，不为复现仿真额外等待20ms；不重复叠加运输和电机响应延迟。pd_reference仅原始PD请求参考，不替代现有电机响应/NET/保护/时序。

编码器delta仍UNKNOWN/null，不能当作0或从日志均值/.12rad中心差猜填。coordinate_adapter.py、motion_coordinates.c/h提供一致坐标边界：q_model=q_report-delta、target_report=target_model+delta、实际previous_report-delta后再按1.5编码；常量delta不改dq。必须先有独立几何标定证据，机械限位/补偿按所属坐标处理。本包不做在线trim、追零或增量解码。

验证入口verification.json、golden_vectors.npz、real_state_physical_PD_golden.npz；真实状态覆盖站立/交接/受扰。目标与PD等价以相同反馈计算，不能把当前请求冒充已经采用的目标。每次同步记录synchronized_capture_fields.json的时间/输入/mean/center/请求/保护/实际采用/PD/力矩/饱和/姿态等字段，同时记录模型与中心SHA。

整包SHA见SHA256SUMS，旧2320包和旧研究包保持。新PPO教师蒸馏会输出另一个名字和教师原中心，不与本C0候选合并能力。
