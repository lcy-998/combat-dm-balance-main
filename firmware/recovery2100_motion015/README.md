> **当前网络替换试验版：** Recovery已改为旧`tjb_recover`（c94bb9f，31→128→64→32→6），腿`1.5×mean`、轮`100×mean`。015 Motion和当前控制配置不变。仿真7组全部交接前失败，不是此前2100成功版；先前2100版可用`ddb5045`。详见[替换报告](../../artifacts/tjb_recover_replacement/REPORT.md)。历史路径名不代表当前网络身份。

# Recovery2100 + 31D015 Motion试验固件

对应分支`feature/rl-supported015-recovery2100`。两套模型、解码和自动交接控制代码均完整包含在仓库。

1. 烧写`recovery2100_motion015.hex`，或将bin写入`0x08000000`。
2. 上电保持原零力/急停模式，待六电机/IMU反馈有效，再使用原非零力使能。
3. 默认先Recovery起身；本体IMU与编码器满足准备条件20ms且Motion目标连续时自动接管。
4. 零力/急停退出。故障锁存后需回零力模式，再使能重新起身。

观察`g_rl_shadow_debug.recovery_mode`由1变0、`handoff_committed=1`确认交接。默认`handoff_source=2`是姿态/构型准备条件，不是触地证明。不得通过写假接触或改编码器零位强行触发。

本次未由agent刷机或驱动电机；真实MCU推理耗时/任务栈高水位尚待目标记录。实机Recovery阶段当前无补偿（代码现状，尚未证明该模型实机不需要补偿）；Motion物理目标实际采用后恢复普通015原有弹簧补偿，按PD5ms更新并保持，最终±40Nm限幅。仿真采用完整V4，不额外叠加补偿。当前旧tjb_recover复测仍0/7，不能据此包宣称稳定起身。完整接口、操作和测试边界见[接入说明](../../docs/RECOVERY2100_HANDOFF.md)。SHA与构建占用见manifest.json。
