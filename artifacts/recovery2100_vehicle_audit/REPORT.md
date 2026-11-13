# 电控兼容与实车前审查

结论：两套网络均为C固化权重，解码按Recovery/Motion明确分流；修复后可进入有保护的实车分步验证。当前没有目标MCU推理截止时间、任务栈高水位和本车力矩/编码器实测证据，不能认证直接无保护落地自动起身。

| 模型 | 活动C权重/推理 | 实际输出解码 |
|---|---|---|
| Recovery2100 | application/chassis/recovery2100/recovery_actor.c：static const float数组、C ELU/Tanh | 固定center+.15a+(reach-.15)a³；轮100a |
| 31D015 Motion | X-CUBE-AI/App/tjb_network_data_params.c：ai_u64中打包FP32，ST C推理 | C0_poly5(height)+.15a；轮80a |

两者运行时都不加载ONNX/Python。Recovery目前是独立C核，不是ST生成图；若要求完全相同的X-CUBE-AI格式，属于另一种运行时集成，当前不冒充已完成。无需仅为“C形式权重”而改图。旧X-CUBE-AI/App/tjb_recover*是128网络历史文件，不是当前2100，从Makefile活动构建中移除，保留历史文件。015生成Motion权重未更改。

## 已定位并修复

1. 模式冲突释放控制权：Recovery会话期间台阶/跳跃标志使执行许可为false，旧recovery_execute清除整个会话，后续可回到旧LQR；修复为RL_FAULT_MODE锁存SAFE零输出，只允许显式零力模式Reset清除。根因/修复定位application/chassis/rl_control.c:186；新增真实控制回归复现旧版失败，修复后通过。
2. 重新使能沿用旧观测：shadow仅依赖recovery_session布尔暖填FIFO；任务未读到停机窗口时，新epoch仍复用旧历史。修复为新epoch暖填；仅确认的Recovery→Motion交接(epoch+1、committed、模式切换)保留FIFO。定位application/chassis/rl_shadow.c:308。真实RLShadowTask新增第15情景，旧版断言失败、修复后通过。

这两项不改变网络、decoder、默认本体准备100ms或正常交接动作。原ffed642固件和旧ZIP已被当前产物取代。

3. 采集窗口提前冻结：1kHz回调下旧采集器在第一次RL输出后约112ms冻结，记录不到0.5–0.84s的接管。改为200Hz采集、Recovery会话在交接或故障触发，保留约555ms后续。定位application/chassis/rl_capture.c:29。新增1100ms实际消费者回归通过。冻结记录保留，读取后可显式调用RLCaptureArm重新采集，不在安全模式自动清掉故障证据。

## 验证

- 电控9/9测试，通过原8组及Recovery完整控制回归；真实shadow15情景通过。
- Python工具集9项无失败，其中6项因本机缺外部原ONNX跳过。
- Recovery真实128状态C golden：mean最大误差4.293397068977356e-7、物理目标最大误差2.288818359375e-5。
- ARM完整构建通过：Flash640156B；DTCM82592B；RAM_D1 59456B。
- IntelHEX逐记录checksum、HEX对BIN数据、初始SP/Reset向量检查通过，见flash_integrity.json。
- 未重新运行物理闭环：此次只修会话生命周期；原100ms七条件闭环仍为6/7，失败发生于交接前。CPU测试不替代实机时序或物理复验。

## 实车剩余边界

- 1ms执行周期严格检查；目标MCU实际调度/双网络探测耗时未验证，若出现漏tick会锁存PD故障归零。应先在受保护条件记录deadline计数、故障码和栈高水位。
- 当前本体准备门只证明姿态/构型接近可接管，不能证明双轮承重；悬空直立也可能自动切Motion。
- 本车实际零位/电机方向、驱动±40Nm量程、轮电流→Nm比例及真实机械弹簧影响仍无本次实测证据。没有新增补偿掩盖这些差异。

试验顺序：先保持零力确认反馈/方向与模式，再在机械保护条件观察Recovery→Motion和故障退出；这些目标检查通过前，不把当前版本称为可直接无保护落地运行。未刷机、未驱动真实电机。
