# Recovery实机对齐015：修复与验证

日期：2026-11-16。审阅基点：Recovery分支c5c8efe，原015参考7f89e6d。只修改feature/rl-supported015-recovery2100；feature/rl-motion31-supported015保持只读。用户要求Recovery阶段固定0m/s、0rad/s、0.34m，交接后恢复正常命令，实机不故意增加延迟，轮限矩4.8Nm。

## 问题与处理

| 问题 | 原实际表现与代码根因 | 已处理 |
|---|---|---|
| 两套执行器 | RLControlExecute在recovery_session时转入独立recovery_execute，交接后仍未离开，和015采用/PD/输出逻辑不同。 | 删除recovery_execute、其状态/缓存及1ms专用ChassisRLRecoveryTick；两技能复用原015 RLControlExecute的采用、PD、补偿和输出。 |
| 人为观测延迟 | RLShadowTask为整个会话维护三槽历史，使用约20ms旧物理反馈。 | 删除历史，Actor及Motion探测只读取队列中的最新有效快照；actor_input_tick_ms为该快照时间，不改写成当前时间冒充新鲜。 |
| 人为执行延迟 | 原会话目标先运输5ms，轮PD力矩再运输7/11ms。 | 删除全部目标/轮力矩FIFO；最新有效目标在下一次正常5ms控制更新直接采用，轮PD结果直接提交。 |
| 轮上限分叉 | 公共PD5.5Nm，会话又裁4.8Nm，预览与实际合同不一致。 | 公共PD两轮明确±4.8Nm；原始电流±16384不提高；轮速Kv0.2、腿40/2不变。 |
| 起身时积累高度请求 | 实际起身高度输入固定.34，但changeL仍累积rl_height_request_m。 | 起身期间速度/角速度固定0、高度.34，内部高度请求也固定.34，忽略积累；交接后正常响应命令。 |
| 1ms专用owner | 在电机发送任务内运行另一执行入口，调度也从原osDelay改为固定拍。 | 统一回5ms ChassisTask拥有策略目标/PD与硬件提交；robot_task.h逐字节恢复原015，没有专用1ms回调。 |
| 六通道事务重复 | 会话和普通路径分别写电机/轮电流。 | 抽取原015事务为ChassisCommitControl，两条生命周期入口共用；交接提交后提示一次，失败不提交模式/采用目标。 |
| 固件与源码身份不同 | 原53d4031 HEX/BIN不含后续CAN/反馈/提示音源码。 | 全部最新源码重新ARM构建，manifest更新每份源及二进制SHA。 |

## 保留与差异边界

- Recovery2100 C权重与三次decoder不变；Motion网络生成代码/权重、Motion decoder、IMU、遥控映射和015电机任务调度不变。
- 两阶段均用原015 RLControlCompensateSpring，零输入调用只记录诊断；最终发送的一份补偿后力矩腿限±40Nm，不重复相加。
- 一次起身生命周期仍由recovery_session标记，以跳过LQR起身、保留故障锁SAFE及记录；此标记不再分派第二个执行器。回零力复位，不在Swing中回退LQR。
- 保留本体20ms准备门、原连续性/epoch/因果previous检查、CAN100ms超时归零和安全重使能、状态回传、交接提示音、Capture v3及集成堆/栈/采集内存容量。这些是显式保留的生命周期/安全/诊断差异，不称整个仓库与015逐字节相同。
- 基点015轮PD5.5Nm在此分支统一改4.8Nm，属于用户明确要求。执行不增加额外固定等待；正常5ms/10ms任务、CAN发送和电机自然响应不等于零时延。
- 原始快照/请求/实际previous/力矩和模式信息分别保存；切换不清previous，不用影子网络输出写执行缓存。物理目标在一次PD/补偿校验成功后与模式epoch同事务采用。

## 实际调用链

ChassisTask（原5ms底盘owner）→CurrentRLSnapshot（最新反馈与命令）→RLShadowTask（10ms最新快照推理，Recovery/Motion各自decoder）→RLControlExecute（同一015采用/PD/补偿）→ChassisCommitControl（同一原015六通道提交）→原电机任务发送。observe_handoff仅更新准备判据。

## 验证结果

- 原0157f89e6d C源码只把轮PD5.5改4.8，1000组×46个结果与新会话Motion逐位一致、最大绝对差0，包括首次采用及无新策略目标时改变q/dq/补偿几何的下一控制tick。不是复制新实现作为参考。
- 9/9主机套件通过；其中普通015/Recovery/接管后Motion1200组相位、推理耗时、owner先后与时钟回绕通过。15类真实RLShadowTask检查输入来自最新快照、无效新帧立即拒绝、Motion候选失效不接管、deadline和过期拒绝。
- 工具9项：3通过、6因仓库外原模型缺失跳过。真实CurrentRLSnapshot检查固定起身命令、不积累高度及交接后正常升降。真实底盘事务检查首个轮PD结果直接输出、模式提交提示一次、下一控制tick失效归零。
- 2100真实128帧golden：mean最大误差4.2933971e-7、物理目标最大误差2.2888184e-5；Physical与Mean后一次Decode逐位相同。权重未更换。
- ARM GCC14.2.1/-Og构建通过：Flash639136B、DTCM81024B、RAM_D159456B。保留项目既有编译警告，不称无警告构建；板端实际deadline/栈高水位未测。
- Ruff0.8.4对修改的Python消费者工具检查通过，git diff --check通过。未更改训练仓正式源码，不新增launcher或训练生命周期。

原始日志、基点逐文件快照/对照结果与golden收据在统一数据根recovery_motion_handoff/shared015_latest_feedback_release。报告只记录本次源码结果，不修改旧run或预算。

## 本轮修改文件

生产：application/chassis/{chassis.c,chassis.h,rl_control.c,rl_control.h,rl_motion31.c,rl_shadow.c}、application/robot_task.h。

消费者：tests/{test_rl_execution.c,test_rl_recovery_handoff.c,test_rl_supported015.c}、tools/{test_chassis_lqr_reentry.py,test_rl_height_chassis_boundary.py}。

文档/交付：README.md、docs/{RECOVERY2100_HANDOFF.md,BRANCH_GUIDE.md}、本报告、firmware/recovery2100_motion015/{README.md,manifest.json,recovery2100_motion015.bin,recovery2100_motion015.hex}。

## 实际验证边界

未刷机、未运行真实电机；本版尚无新实机或闭环物理仿真成绩。旧完整V4下2100独立控制/交接6/7属于原冻结执行合同，不能直接宣称本版能力已通过。仿真继续在原合同中模拟观测/执行不确定性，不在电控再加一次队列，也不额外叠加015补偿。
