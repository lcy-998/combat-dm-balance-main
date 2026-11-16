# Recovery2100起身与015 Motion共用实机执行链

更新：2026-11-16。本分支`feature/rl-supported015-recovery2100`以远端015 `7f89e6d`为参照。当前Recovery为2100蒸馏小模型，Motion为原015网络；两套均为31→256→128→64→6、ELU/Tanh、C权重。本版删除独立`recovery_execute`和实机人为延迟，保留两阶段015弹簧补偿。

## 当前执行定义

```text
5ms ChassisTask
  → CurrentRLSnapshot：最新已收到且有效的六电机/IMU反馈
  → 10ms RLShadowTask：最新快照→Recovery或Motion网络→对应decoder
  → RLControlExecute：最新有效目标立即采用→最新反馈PD→015补偿→最终限矩
  → ChassisCommitControl：六通道一次本地提交
  → 原015电机任务：按正常发送路径输出
```

策略输出在下一个正常5ms控制更新采用，不额外等待。电机/CAN通信、任务调度及硬件响应仍有自然时延，不称实机零延迟。没有刻意添加Actor20ms、目标5ms或轮力矩7/11ms队列，也没有用旧帧替代最新有效帧。

Recovery和Motion共用原015的目标采用、因果previous校验、PD、补偿、限矩与硬件提交。`recovery_session`仅标记一次自动起身生命周期及SAFE故障归属，不选择另一套执行器。`observe_handoff`只观察准备条件，不计算PD、不输出力矩。1ms专用`ChassisRLRecoveryTick`已删除，`application/robot_task.h`与015基点逐字节相同。

## 起身命令与交接

1. 保持原零力／急停模式，等待六路电机及IMU有效。安全模式下自动arm，不主动开启硬件。
2. 原非零力使能后由Recovery起身，跳过传统LQR起身与原Motion启动等待。
3. Recovery阶段硬编码vx=0 m/s、yaw=0 rad/s、height=0.34 m。升降拨杆和高度API不会积累待执行高度请求；内部高度请求在起身期间保持0.34 m。
4. 本体准备条件连续满足20ms后，使用同一最新快照只读推理Motion。候选必须与实际previous因果一致、epoch正确、足够新鲜且物理目标连续。
5. Motion候选在同一个控制更新经过PD／补偿／限矩全部成功后，提交模型切换、目标与epoch。补偿不切换、不重复；不清零previous，不插值、不排队。
6. 交接后恢复原015的速度、转向和升降命令处理。交接候选使用起身时0/0/0.34命令；后续快照按正常命令更新。每次起身到Motion只切换一次，不自动切回Recovery。
7. 交接提交时请求提示音；底盘`standup_flag`按recovering状态回传。捕获记录在当前控制tick标记交接，便于判断震荡在切换前还是后。

## 两种输出合同

| 项目 | Recovery2100蒸馏小模型 | 015 Motion |
|---|---|---|
| C权重 | `application/chassis/recovery2100/recovery_actor.c` | 原X-CUBE-AI `tjb_network`，与015基点逐字节相同 |
| 腿目标 | 固定中心三次绝对目标，边界±1.5 rad | C0五次高度中心＋0.15×mean |
| 轮速目标 | 100×mean rad/s | 80×mean rad/s |
| skill25–27 | 0,1,0 | 1,0,0 |
| 起身／运行命令 | 0/0/0.34固定 | 原015正常命令 |

Recovery腿公式：`d=mean; R=d>=0 ? upper-center : center-lower; q=center+0.15*d+(R-0.15)*d^3`。
中心为`[-0.1309740543,-0.1511767805,+0.1309740543,-0.1511767805]`rad。不能使用线性1.5×mean，也不能使用Motion残差decoder；每个目标只解码一次。

输入均31D，保留真实dq：腿q与previous腿除1.5 rad，previous轮除100 rad/s，dq乘0.05。输入尺度不随输出变化。previous仅记录本地实际采用的物理目标；首次采用前使用与015相同的q/dq初始化代理并明确标记INIT_PROXY，影子推理不更新执行缓存。模型left对应几何右，不交换网络数组。

教师SHA256：`5aec83df5644ae1b96f4ed012be41d41cb15c4729bbc7b632af347912d3d12a9`。
部署Actor SHA256：`660a5686251171fba9835c28be292868e4afaec3cdd8609e7346bf6c9b03620d`。
源部署包：`/home/kellen/wheel_leg_training_data/2026_11_13__recovery2100_mujoco_rise_r2/deployment_v3`。恢复的是蒸馏小模型，不是直接运行大教师；本轮没有训练。

## PD、补偿与安全

- 腿PD40/2、轮速Kv0.2。**公共PD层明确腿±40 Nm、轮±4.8 Nm**，预览与实际执行一致。015参考分支仍保持原样；本集成分支按用户要求将其5.5 Nm轮PD上限改为4.8 Nm。
- 两阶段同一`RLControlCompensateSpring`：PD限幅→motor顺序映射→加一次015补偿→最终腿±40 Nm。零输入函数调用只取得诊断项，最终输出不再加诊断项。
- 保留原始轮电流限幅16384及方向转换；源码Nm／电流换算不冒称已做轴端力矩实测。
- 最新反馈和策略仍检查新鲜度≤24ms、六轴有效、IMU有效、有限值与对应decoder。异常后会话锁SAFE六通道归零，不在起身中回退LQR；需显式零力复位。该安全归属是保留的生命周期差异，不是第二套PD执行器。
- 保留远端CAN100ms超时归零与恢复后先接收零力再使能的保护；与015基点的差异单独说明，不悄悄移除。
- 正常任务栈／堆的集成容量和采集RAM保留；未改变IMU采集、电机标定、遥控映射或Motion权重。

本体门使用最新电机q/dq、IMU重力与角速度及真实接收时间戳。距直立轴≤0.35rad、gyro范数≤3rad/s、四腿速度≤8rad/s、四腿距Motion0.34m中心≤0.50rad，持续20ms；目标跳变腿≤0.4rad、轮≤40rad/s。dwell按最旧必要反馈时间推进，重复同一反馈不能累积。该门不是触地证明，不使用假法向力或仿真高度。配置在`application/chassis/rl_recovery_config.h`，阈值未收紧。

## 仿真与实机分工

Isaac／MuJoCo训练与评估可以按冻结spec增加观测和执行延迟，模拟真实系统不确定性；实机不把这些仿真队列再执行一遍。完整V4＋NET仿真不额外添加015补偿；实机保留原软件补偿及真实弹簧。两者的完整系统等效仍需采集版本与原CSV绑定，不由本次软件修改证明。

历史完整V4记录中，同一2100小模型独立控制及20ms本体交接均6/7；旧tjb比较版0/7。那些实验使用其当时冻结执行合同，**不代表本版最新反馈实机路径已做新的闭环验证**。历史报告及原始数据保留，不改写其失败和能力记录。

## 验证与交付

正式实机调用链在上方；不再保留并行Recovery PD实现。针对性验证包括：

- 原015 `7f89e6d`仅把轮PD限矩5.5改4.8后，与新会话Motion1000组输入、每组46个采用目标／PD／补偿／最终输出值逐位一致；第二个控制tick无新目标、改变q/dq与补偿几何仍一致。
- 三类路径（普通015、Recovery、接管后Motion）共1200组Actor／PD相位、推理耗时、owner先后顺序和uint32回绕；9/9主机套件含15类真实RLShadowTask路由／最新输入／耗时／错误情景。
- 真实`CurrentRLSnapshot`测试起身固定0/0/0.34、拒绝积累高度请求、交接后恢复命令；真实底盘输出选择与共享提交测试首个目标同tick生效、双轮即时输出、交接提示一次和失效锁零。
- 2100权重及decoder保持原有128帧真实golden身份；非端点／中心探针区分三次与线性解码。

运行方式：

```sh
sh tests/run_rl_host_tests.sh
PYTHONDONTWRITEBYTECODE=1 /home/kellen/anaconda3/envs/issaclab/bin/python -m unittest discover -s tools -p 'test_*.py'
make -j12 SHELL=/bin/sh \
  GCC_PATH=/home/kellen/.local/opt/xpack-arm-none-eabi-gcc-14.2.1-1.1/bin \
  OPT='-Og -fstack-usage'
```

HEX／BIN及源码SHA见`firmware/recovery2100_motion015/manifest.json`，bin加载地址0x08000000。旧53d4031固件不含后来CAN／反馈／提示音源码变更，不能冒称最新；本轮重新构建全部最新源码。

本轮原始检查证据位于数据根`recovery_motion_handoff/shared015_latest_feedback_release`，源码内修复清单见[015对齐审阅](../artifacts/recovery2100_015_alignment/REPORT.md)。未刷机、未驱动电机；板端推理deadline、栈高水位和真实闭环能力仍需实测。本轮不通过额外叠加补偿仿真来代替实机验证。
