# 2026-11-09 RL 机体 heading 估速修复

已修改桌面2源码，未烧录。当前D5/25D/6D网络接口保持；本次修的是obs6所用速度，不是重新训练策略。

## 修复原因和方法

训练速度是root刚体COM点的世界线速度在水平heading的投影。此前RL直接使用传统控制的`R/2*(-dqL-dqR+wLegL+wLegR)`，它不是同一个物理量。LQR的KF能配合其控制器，不证明这个代理量适合D5的瞬态反馈。

新的`RLHeadingVelocity`使用四腿实际角/角速度、两轮相对编码器速度和现有IMU姿态/角速度：

1. 闭链正运动学求轮心位置及相对机体速度（保留0.21/0.25m杆长和现有装配分支）。
2. 求轮电机安装的**下连杆D→C**角速度，不再把虚拟腿O→C角速度当作电机安装件角速度。
3. 加入机体刚体转动项，将左右轮独立转换到同一个root参考点，再平均并投影到水平heading。
4. policy是forward/right/up反射坐标，角速度和叉乘沿已有det−1契约处理；没有翻改IMU观测或轮输出符号。

标称参考点相对两髋轴中点为forward/right/up=(0.0055423,-0.0052237,0.1277691)m，来自训练资产的root COM位置；它是选定参考点，**不声称已经辨识实车质心**。轮半径0.06m，半轮距0.2215m。参数在新header集中定义。

新公式不读取真实root速度，不增加低通滤波或队列。LQR的s_dot、WorldXSpeedKF、里程计、四腿PD、弹簧补偿、轮阻尼/库仑补偿、16384限流和action/history契约保持原样。计算耗时仍需H723上实测。

同时修复异常路径：C的fminf/fmaxf可将NaN吞成限幅值，因此RLShadowBuildObservation现在在clip前检查heading速度与命令是否有限。估速失败返回NaN并让已有无效输入处理拒绝输出，而不是回退旧代理量。

## 精确修改文件

- `application/chassis/rl_heading_velocity.h`：新增无状态完整运动学估速。
- `application/chassis/chassis.c`：RL snapshot填入新速度；保存原始代理速度供对照。
- `application/chassis/rl_shadow.h`：末尾增加估速诊断字段，原字段顺序保留。
- `application/chassis/rl_shadow.c`：发布估速诊断字段。
- `application/chassis/rl_observation.c`：clip前有限性检查。
- `tests/test_rl_heading_velocity.c`：新增前向滚动、纯转弯、无效输入和仅obs6改变测试。
- `tests/test_rl_observation.c`：增加NaN/Inf速度与命令拒绝测试。
- 本说明`FIXES_20261109_RL_SPEED.md`。

新增估速是header-only，无需添加Keil/CubeIDE源文件列表。由于snapshot/debug结构追加字段，必须完整重编译并用新的ELF读取Ozone符号；不能沿用旧偏移。保留实际第三版D5的CubeAI生成网络/权重，本次没有替换它们。桌面2原有生成报告标记不代表实车当前网络，不要在合入本修复时意外换回旧权重。实车已验证的补偿开关也应保持。

## 验证结果与边界

同一D5、相同初态/噪声/DR/延迟，4.89Nm peak、5.5 action scale、PD60/2、spring4.31、physics2ms/policy8ms：

| 工况 | 旧代理估速 | root真值基线 | 新传感器估速 |
|---|---:|---:|---:|
| 待机/.3/.5各16副本，首12s存活 | 0/48 | 48/48 | 48/48 |
| 0→.3→0→.5→0，首12s存活 | 0/16 | 16/16 | 16/16 |
| -.4、.5±.3yaw、1±.8yaw、1.5直行，各8副本 | 未在此布局重测 | 48/48 | 48/48 |

以上是副本，不是112个能力点或完整部署认证。静态待机/.3/.5新估速最差稳态pitch峰峰值1.535/.966/.460°，速度峰峰值.0482/.0325/.0123m/s。待机仍有漂移和小周期波动，D5的起停瞬态及小yaw欠响应仍然存在；两种速度源基线都有，不包装成已全修好。新估速并非每一指标都优于真值输入，例如部分转弯误差稍增，但全部48补充副本存活、稳态pitch均≤3°。实机demo和Ozone不同步，本实验没有把二者拼成同一时间线，也不是10秒LQR接管的精确重放。

编译后的C与Torch公式10,000随机状态对照，最大误差1.4305e-6m/s；6组host C测试通过；相关Python测试11项通过。三个修改的固件C翻译单元用原Makefile includes/defines做host syntax检查通过（原有PI重定义warning）。环境无arm-none-eabi-gcc，**未完成ARM链接/固件镜像编译，未烧录**。

## Ozone复核字段

在原有obs/action基础上记录`g_rl_shadow_debug`的：
- `heading_vx_m_s`：现在实际进入obs6的新估速。
- `heading_vx_legacy_m_s`：旧的未滤波代理估速。
- `heading_vx_sides_m_s[0/1]`：左右轮独立推回同一root点的估速。
- `heading_vx_estimator_valid`：仅表示数学输入有效，不保证无打滑/接地。
- `snapshot_seq/snapshot_tick_ms/inference_tick_ms/publish_seq/rl_active`：配拍与识别真正接管时段。

正常关系仍是obs6=clip(2.1*(target_vx-heading_vx),±2)。比较时须使用同一完整publish_seq；不要把100Hz CSV相邻两行当125Hz相邻推理。新估速不解决打滑、离地和异步CAN反馈年龄，仅修正几何速度定义。实车效果以新日志复核，不能从仿真存活保证。

所有改前文件、SHA、精确patch、命令、同源配对结果和曲线均保存在：
`/home/kellen/桌面/11 (1)/11/wheeled-legged_RL/artifacts/speed_estimator_fix_20261109/`。
原始对比曲线：`/home/kellen/桌面/RMUC训练曲线/speed_estimator_fix_20261109/`。
