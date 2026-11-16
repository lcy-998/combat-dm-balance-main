# Recovery2100＋015 Motion共用执行器试验固件

当前分支feature/rl-supported015-recovery2100。Recovery是2100蒸馏31→256→128→64→6 C网络，三次绝对腿目标、轮100×mean；Motion为原015 C网络，C0poly5(height)+0.15×mean、轮80×mean。

两阶段同一5ms底盘owner、最新反馈与最新有效目标、公共PD40/2/.2、015弹簧补偿一次，最终腿40／轮4.8Nm。已删除独立recovery_execute、Actor20ms历史、目标5ms队列与轮7/11ms队列，电机任务调度恢复015原样。自然任务／通信／执行时延仍存在。

1. hex直接烧写，或bin写入0x08000000；文件SHA与本次源SHA在manifest.json。
2. 先保持原零力／急停模式，六电机与IMU有效后使用原非零力使能。
3. 起身命令固定速度0、角速度0、高度0.34m，不积累升降拨杆请求。
4. 本体准备20ms且Motion新鲜候选连续时交接；之后正常响应015命令，交接提示音只请求一次。
5. 故障锁零，显式零力复位；CHASSIS_BOARD保留CAN100ms超时／安全重新使能保护。

观察g_rl_shadow_debug.recovery_mode由1变0与handoff_committed=1；只有本体姿态准备，不能将提示音当作触地认证。Capture v3按200Hz记录，在交接／故障触发后保留约555ms，原始格式／来源字段保留。

本次包含最新远端CAN、反馈和提示音源码重新构建；旧53d4031固件不代表这些更新。主机、等价性、golden与构建结果见manifest及[接入说明](../../docs/RECOVERY2100_HANDOFF.md)。旧完整V4下6/7属于原冻结合同，本轮未新跑闭环仿真或刷机，实机能力／MCU耗时／栈高水位未验证。V4仿真不额外叠加软件补偿，015参考分支未修改。
