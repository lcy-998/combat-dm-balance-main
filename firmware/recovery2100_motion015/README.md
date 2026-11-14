# Recovery2100 + 31D015 Motion试验固件

对应分支`feature/rl-supported015-recovery2100`。两套模型、解码和自动交接控制代码均完整包含在仓库。

1. 烧写`recovery2100_motion015.hex`，或将bin写入`0x08000000`。
2. 上电保持原零力/急停模式，待六电机/IMU反馈有效，再使用原非零力使能。
3. 默认先Recovery起身；本体IMU与编码器满足准备条件20ms且Motion目标连续时自动接管。
4. 零力/急停退出。故障锁存后需回零力模式，再使能重新起身。

观察`g_rl_shadow_debug.recovery_mode`由1变0、`handoff_committed=1`确认交接。默认`handoff_source=2`是姿态/构型准备条件，不是触地证明。不得通过写假接触或改编码器零位强行触发。

本次未由agent刷机或驱动电机；真实MCU推理耗时/任务栈高水位尚待目标记录。模型物理弹簧/力矩标定沿平台现状，未新增补偿。完整接口、操作和测试边界见[接入说明](../../docs/RECOVERY2100_HANDOFF.md)。SHA与构建占用见manifest.json。
