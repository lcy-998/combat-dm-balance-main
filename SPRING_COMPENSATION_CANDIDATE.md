# 2026-10-22 部署弹簧差额补偿候选：未通过实录0.5deg门槛

2026-11-10 合并说明：经用户确认，保留本地部署行为，`RL_SPRING_DIFFERENCE_ENABLED` 默认改为 1，仍可设为 0 关闭。以下内容是此前候选版本的历史记录，实车验证状态未因此改变。

用户确认：部署端补偿、保持策略不变；前100ms四关节实录vs仿真最大误差各<=0.5deg。

改动：桌面2 application/chassis/rl_spring_compensation.h、chassis.c ApplyRLJointOutput、tests/test_rl_spring_compensation.c。RLSpringDifferenceTorque从旧LQR scalar*sin角差得到S_L/S_R，电机encoder坐标补偿[4.31-S_L,+S_L,S_R-4.31,-S_R]。进入hostPD feedforward，在总40Nm限幅之前，不修改action/history。对应D5已核对gas_spring_torque_nm=4.31，不是10.775。RL_SPRING_DIFFERENCE_ENABLED默认0，未烧录，原LQR逻辑不变。

101组左右角度的净力矩符号/幅值核对、非法输入归零、总PD力矩限幅测试通过，git diff --check通过。未发现ARM编译器，未整机编译。

同实录动作、同初态、C代码PD/consumer/补偿运行0.5s；训练原模型active-only4.31与假设实机双端旧律+补偿：前100ms四关节最大角差[.000506,.000563,.000575,.000705]deg。这只是同一假设模型加其逆的内部一致性验证，不能证明实机弹簧正确，也不保证饱和/模型误差下等效。

未补偿旧律仿真与旧Ozone前100ms最大差[2.1692,2.0110,1.1803,.5433]deg：全部未过0.5。此前hip4平均误差.204deg不能替代最大误差。补偿后模型接近训练模型，自然不会继续贴合未补偿旧实录，其对旧记录最大差[4.3247,8.9800,6.7182,10.5643]deg；不是相同实验条件的实机验收。

要完成用户门槛，需先验证被动模型/执行时序，或获得安全验证后的补偿实车新记录；不能通过篡改初态/力矩逐帧拟合把这段旧记录包装成验证通过。本次不启用默认补偿，不改变训练主线物理或交付ONNX。备份chassis.before.c，全部CPU命令/JSON/测试证据保留。
