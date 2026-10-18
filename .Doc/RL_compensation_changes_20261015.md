# RL轮控修订与验证（2026-10-15）

基线HEAD：5b262d7bbf80cb9a4bf65cc40788064bdb8af492，与此前feature/rl-input-shadow审计一致。开始时tracked文件干净，原有.deep-copilot未动。未刷写、未推送。

## 执行契约

物理前进为正，在Nm域等效为：

```
tau_drive = 5.5 * clip(action, -1, 1)
tau_motor = clip(tau_drive - Dfb * omega + Cff * tanh(omega / 0.1),
                 -16384/K, 16384/K)
K = 19 / (0.00036 * (268/17)) = 3347.844... counts/Nm
```

物理左/右：Dfb=[0.0494515,0.05133043] Nm·s/rad，Cff=[0.053699,0.051843] Nm。对应辨识B=[0.0025485,0.00066957]、C=[0.054699,0.052843]，未饱和时总黏性约.052、残余库仑约.001。实机机械项自然存在，固件不再额外减B/C。相同常数来自当前fit_compensated训练profile，不使用旧的sign切换库仑律。

16384对应约4.893896Nm/轮，不是5.5Nm；action scale仍5.5。持续热能力未经验证，旧3Nm只作参考。原先RLMap和Blend的8192提前clip移除，驱动+补偿合并后才受电流预算限制。发送前相同16384保护夹紧不改变已饱和值；输入非有限时输出0。禁止其他上游再次限流8192或再次补D。

## 调用链

- rl_output_compare：原v8六维输出语义/腿±1 clip保留，轮5.5action scale保留。
- RLControlMapOutput：腿映射保留，轮转换到电机电流坐标但不提前clip；左电机负电流为前进，右为正。
- ChassisTask：原整帧RL queue保留；左右驱动与RL模式在critical section原子发布。
- MotorControlTask每次调用ChassisWheelControlTick，然后DJIMotorControl；轮目标保持，补偿按新轮速每1ms重新计算。轮速和obs取相同measure.speed_aps滤波来源，轴侧、减速比、deg→rad一致，物理前进坐标为obs轮轴坐标取反。不是另加滤波器。
- critical section仅复制目标/模式/速度；tanh在外部计算，避免长时间屏蔽中断。
- 腿DMMotorControl仍每两次motor tick执行；MCU shortest-angle PD60/2、±40Nm，电机MIT内部KP/KD保持0，不重复PD。
- 轮close_loop与outer_loop明确OPEN_LOOP，直接current feedforward；不再经过原零增益速度PID。
- 非RL的LQR分支保留原摩擦补偿，仅电流预算升级16384；RL启用时旧摩擦补偿不执行。

## 其他已修问题

- rl_shadow直接使用remote_vx，移除积分位置目标/PID覆盖；位置函数保留供对照测试，不进入RL命令链。
- heading速度轮deg/s先转rad/s，再加腿FK rad/s，修正原本把腿项再次乘DEGREE_2_RAD的问题。
- snapshot/output过期阈值24→12ms；这是软件拒收而非端到端时延证明。原机器人任务周期与调度抖动仍要测量，过期频繁会退出RL，不应通过放宽阈值掩盖。
- DMmotor.h大小写改为实际dmmotor.h，便于Linux集成检查。

## 变更文件

application/chassis/{chassis.c,chassis.h,rl_control.c,rl_control.h,rl_shadow.c,wheel_friction.c,wheel_friction.h}
modules/motor/motor_task.c
tests/{test_rl_control.c,test_wheel_friction.c,test_wheel_rl_compensation.c}
本说明文件。

## 验证与剩余项

8组host tests通过：RLcontrol、legacy friction、新RL补偿、observation、output、DM PD、position helper、watchdog。新补偿测试含左右符号、零速零偏、±30rad/s、5.5驱动在同向补偿后不被提前削顶、反向饱和16384、连续扫速、NaN置零。

chassis/rl_shadow/motor_task在Makefile相同宏与include目录下host GCC语法检查通过；chassis有原有PI重定义warning。git diff --check通过。证据build/host_tests/integration_syntax.json。

本机无arm-none-eabi-gcc，未生成完整ARM固件，未测CubeAI推理耗时/CAN周期。原Makefile目录命令是Windows语法；未为此改造整套构建。上车前用原Windows工具链编译，核对1ms轮/2ms腿/8ms策略实际时间戳、12ms过期计数、左右轮符号、反馈电流和温升。不会据host测试称sim2real通过。

本次不改网络、不用新教师覆盖v8、不改腿零位/安装符号，不凭猜测加腿目标低身偏置或部署slew。RL启用仍是原立即切换行为，后续应以真实切换轨迹验收；不能认为轮补偿修复自动消除所有切换/停车失稳。
