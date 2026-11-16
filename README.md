> **当前2100＋015共用执行器版（2026-11-16）：** 2100蒸馏小模型负责起身，随后015 Motion接管；实机不额外添加观测或执行延迟，两阶段共用原015的5ms目标采用／PD／补偿，轮限矩统一4.8Nm。


# 轮腿电控：先选对分支

**当前要试“Recovery起身→自动交给Motion”，请选择 [`feature/rl-supported015-recovery2100`](https://github.com/lcy-998/combat-dm-balance-main/tree/feature/rl-supported015-recovery2100)。**

该分支以015 Motion为基础，包含Recovery2100小模型、31D256 Motion、本体感知20ms准备门、自动交接及C权重。已删除独立recovery_execute和实机20/5/7/11ms队列，保留远端CAN保护与交接提示；发布文件的SHA以manifest为准。

- [固件HEX/BIN、下载和操作](firmware/recovery2100_motion015/README.md)
- [输入、两种decoder、时序、自动交接与故障退出](docs/RECOVERY2100_HANDOFF.md)
- [本版015对齐修复清单与验证边界](artifacts/recovery2100_015_alignment/REPORT.md)
- [历史实车前审查](artifacts/recovery2100_vehicle_audit/REPORT.md)
- [完整分支用途与本机目录对照](docs/BRANCH_GUIDE.md)

实车状态：本版验证主机回归、真实状态golden和ARM构建；限定条件仿真属于旧冻结执行合同，不能直接认作本版实机成绩；尚未由本会话在实车上验证。只能进入有保护的分步试验，不能把它称为已认证可直接无保护落地运行。
