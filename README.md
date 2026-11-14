# 轮腿电控：先选对分支

**当前要试“Recovery起身→自动交给Motion”，请选择 [`feature/rl-supported015-recovery2100`](https://github.com/lcy-998/combat-dm-balance-main/tree/feature/rl-supported015-recovery2100)。**

该分支以015 Motion为基础，包含Recovery2100小模型、31D256 Motion、本体感知100ms准备门、自动交接及C权重。功能代码审查修复提交为`208910e`，后续导航文档提交不改变固件；发布文件的SHA以manifest为准。

- [固件HEX/BIN、下载和操作](firmware/recovery2100_motion015/README.md)
- [输入、两种decoder、时序、自动交接与故障退出](docs/RECOVERY2100_HANDOFF.md)
- [实车前审查、已修bug与尚未实测边界](artifacts/recovery2100_vehicle_audit/REPORT.md)
- [完整分支用途与本机目录对照](docs/BRANCH_GUIDE.md)

实车状态：已通过主机回归、真实状态golden、ARM构建及限定条件仿真；尚未由本会话在实车上验证。只能进入有保护的分步试验，不能把它称为已认证可直接无保护落地运行。
