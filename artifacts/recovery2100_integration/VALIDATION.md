# 验证收据

基底：`origin/feature/rl-motion31-supported015` / `786a2c56`。所有015生成Motion文件无diff；未刷机、未真实驱动电机、未推送远端。

- `sh tests/run_rl_host_tests.sh`：9/9，通过原8项与新增Recovery控制回归；新增测试还执行真实RLShadowTask的14个输入/路由/延迟/故障/wrap情景。日志 `host_tests.log`。
- 主机重编译Recovery Actor，128条部署包真实状态golden：mean最大误差4.293397068977356e-7，物理目标最大误差2.288818359375e-5。
- 实际ARM完整构建通过；GCC14.2.1，`-Og -fstack-usage`。Flash638812 B/1048576 B；DTCM82560 B/131072 B；RAM_D1捕获缓存57920 B/327680 B。没有实测MCU推理时间或任务栈高水位。日志 `build.log`；binary/map在本地忽略目录`build/`。
- ARM静态栈：RecoveryActorMean2064 B、RLShadowTask552 B；ChassisRLRecoveryTick768 B、CurrentRLSnapshot480 B。不同调用链不能简单全部求和；任务分配为shadow8192 B、motor4096 B。新增RTOS heap32768 B。下位机截止时间仍须目标实测。
- Python实际底盘/LQR重入与高度边界2项通过。测试准备器随新增gate链接、CurrentRLSnapshot参数接线更新；同时修正015旧高度步长测试，使用当前0.048m/s×0.005s，而非过时的0.002×0.03。
- 完整Python工具集14项：8通过、2skip、3失败、1error。失败均属于015已有权重工具/旧ONNX golden与最新256网络不一致：`tools/update_xcube_ai_weights.py:28`仍绑定128/64/32，相关测试仍读取旧模型。干净786a2c5复跑同样3失败+该error，另有1个旧高度准备器error。完整日志与基底日志分别 `python_tools.log`、`baseline_python_tools.log`。没有修改权重转换工具或覆盖015网络。
- 仿真与训练仓CPU结果见 `docs/SIMULATION_RESULTS.md`：development7组×3方案，6/7；200ms方案接管后6/6保持10秒。portable2011通过/31skip，架构/Ruff通过。

完整固件、golden和闭环分别验证不同边界；目前不是任意倒地实机成功认证。硬件实际高度与独立双轮支撑生产者缺失，缺省UNKNOWN阻止自动交接。
