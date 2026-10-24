# Motion31 执行链修复与主机验证

2026-10-25。以 `bff439f` 为修复来源；本修复供电控审查合并，未刷写、未做带电测试。
固定网络仍为Motion500小模型，未修改生成网络权重。该工作是执行接线修复，**不是实机已经站稳或可直接刷写的声明**。

## 当前实现

- `RLShadowTask`每10ms读取完整快照、构造31D并推理，仅发布物理目标/seq/epoch；不计算持续执行的PD矩，不再回灌shadow预测。
- `RobotTask`改为绝对5ms调度。正常底盘路径用同一份本周期反馈调用`RLControlExecute`，即使没有新Actor目标也重新计算PD。增益仍40/2/.2。
- `RLControlFillPrevious`在RL接管后读取本地PD采用缓存；接管前明确标为q/轮dq初始化代理，不能当成LQR曾执行的目标。
- 推理允许、交接资格、真实owner分开。目标必须属于当前epoch并与上一采用seq因果一致。六轴先选owner和输出，再在同一critical section写入；CAN各帧依然异步，未宣称远端驱动同时接受。
- 故障拒绝继续RL并沿用既有LQR路径，RL错误锁存；只有显式`CHASSIS_ZERO_FORCE`复位清除。具体故障退控是否适合实机须由电控负责人复核，不能把LQR回退当作所有异常下均安全。
- 有限非法角度与异常重力拒绝，不再clip输入继续；超大有限数据增加序列化保护。动作尺度/PD/电机限矩不变，轮电流仍夹8192。
- CAN驱动增加反馈接收tick/seq；IMU记录滤波更新完成tick/seq，均不是传感器采集时刻。未知年龄不会被任务时间伪装。当前CAN IRQ priority=5与FreeRTOS最大系统调用priority=5匹配，快照在critical section中复制；异步传感器仍各保留自身年龄。
- 原有getPhi使用`RL_KINEMATICS_HOST`条件隔离后可在主机编译，同一函数供MuJoCo薄适配调用；未复制五杆公式。

## 同步记录

`g_rl_shadow_debug`改为完整工作副本提交，`frame_begin==frame_end`且偶数才可使用；一份Ozone多变量CSV仍不自动保证读数是同一帧，优先使用下面冻结RAM记录。

`g_rl_capture`由控制任务单生产者写入，`rl_capture.h`固定32位字段，默认128条。当前每条440字节，总56368字节；200Hz下约.64秒容量，默认保留约.08秒触发前及.56秒接管后。它足以捕获最早震荡，不冒称已覆盖完整倒地。增大容量必须先确认最终ELF和FreeRTOS堆/栈余量。

首次RL接管或故障触发，记录到指定条数后冻结，**只冻结采集，不自动停止电机**。记录包含请求/采用seq、完整obs31/mean6、PD实际q/dq、PD矩/补偿/最终本地命令、CAN接收/IMU完成tick、驱动报告的腿矩与轮电流raw。driver反馈不是独立标定力矩，轮raw没有被伪称为安培。`valid_mask`标明哪些policy/PD字段真实存在，缺失不是0测量。

负责人在安全状态下导出`g_rl_capture`完整二进制，并附：

1. 本次ELF/build、源码版本及模型身份。
2. 导出地址、字节数、`sizeof(RLCaptureRecord)`和容量（若改过编译宏必须说明）。
3. 8192电流单位、Kt/减速比/允许持续及峰值电流的确认结果，未知写UNKNOWN。
4. 测试条件、实际owner切换与视频时间对齐（如可得）。

不得在动态平衡时为读取RAM而halt MCU。当前没有新增自动刷写、调试器连接或电机测试脚本。

## 主机验证与训练侧对接

合并后可直接在本仓库根目录执行，不依赖训练仓库、Python、Isaac或电机：

```sh
sh tests/run_rl_host_tests.sh
# 可选指定主机C编译器（仍然不是ARM固件构建）
CC=clang sh tests/run_rl_host_tests.sh
```

需要POSIX shell、主机C11编译器和libm。脚本只在系统临时目录生成可执行文件和合成capture，退出时删除自己的临时目录；不连接设备、不修改工程源码或写入训练目录。五项分别覆盖编码/解码/PD数学、原弹簧补偿、实际采用/200Hz PD/故障状态机、冻结RAM记录、真实RLShadowTask边界。最后一项仅RTOS和AI运行库用桩，不是目标板ST推理实测。

训练项目的`scripts/validate_motion_firmware.py`编译并运行：原数学测试、原弹簧测试、新执行状态机、新RAM采集、真实RLShadowTask边界（仅RTOS/推理运行库为桩）。同时生成`.so`供`native_stand.students.firmware_control`调用。

主机桩不代表STM32运行库测试。已用ARM Clang对10个真实修改路径完成语法及对象编译，静态帧示例ChassisTask600B、RLShadowTask576B、RLControlExecute168B；这不是任务完整调用链/ISR/ST推理库的最大栈。当前主机没有`arm-none-eabi-gcc`，完整Makefile/库链接和最终ELF内存布局、高水位需在电控环境验证。**请勿仅凭主机测试通过直接替换实机固件。**

电脑MuJoCo已复用真实C内核完成6组短对照及14组30秒零命令时序矩阵；共456物理秒均未倒，但仍约.269m/s漂移。理想抵消/无实体弹簧两类是明确边界假设，不能证明真实补偿正确或原地站稳。共享输入/PD数值对照通过也不能替代目标板调度和反馈年龄测量。

计划、原审计、机器结果及下一步统一见训练项目：

- `docs/MOTION_SIM2REAL_EXECUTION_ALIGNMENT_PLAN_20261024.md`
- `docs/MOTION4995_SIM2REAL_GAP_20261021.md`最新入口
- `artifacts/2026_10_24__motion_execution_alignment/`

轮电流按现有公式反算8192约2.447Nm，不是实测结论；不为追平仿真5.5Nm擅自加电流。新MuJoCo对照是声明假设下的单环境有限诊断，不能代替实机接管验证或完整站立证书。

## 电控合并审查与上机前待确认

本提交会改变正常LQR路径到RL的控制交接和故障锁存，不能只替换生成网络文件。请整体合并`chassis`、驱动接收时间、IMU完成时间及任务调度修改；`rl_capture.c`已加入Makefile源清单，其他IDE工程若单独维护源清单也须纳入该文件。`legalgorithm.c`的`RL_KINEMATICS_HOST`只供主机编译，正常固件不定义它；`tests/host`中的桩头文件禁止加入固件include路径。

保持原有5秒腿长ramp和随后5秒LQR等待，满足资格后才尝试采用当前epoch的完整RL目标。这是现有自动接管条件的修复，不是新增人工ARM开关，也未证明首帧目标/力矩无跳变。影子模式只预测，不能据`rl_active`认定六轴已经RL；真实归属读`controller_owner`及capture的owner/采用seq。

优先复核以下项目：

1. 真实工具链完整构建、map中的RAM/Flash、AI工作区和FreeRTOS栈高水位。新增RAM capture约56.4KB；不能将单函数静态栈估计当整个任务上限。
2. 100Hz推理和200Hz控制的实际周期/耗时；当前绝对调度不保证任务不超期。CAN接收和IMU完成时间不是传感器采样时间。
3. 电机ID/几何侧/编码器零位和方向、角度分支、IMU安装与坐标；源码内部一致不等于实体已标定。模型ch0/1/2为几何右，ch3/4/5为几何左。
4. 首次接管的`INIT_PROXY`只是当前腿q/轮dq代理，不是LQR历史目标。真实RL采用后才能标`ADOPTED`；软件六轴提交也不是远端驱动同时接受。
5. 异常时当前实现保持既有LQR路径并锁存RL故障，显式ZERO_FORCE清除。请审查LQR回退是否适用于实际失衡状态，不能默认它在任何故障下均安全。
6. 轮raw8192单位、Kt/减速比、允许持续/峰值电流及最终生效限流；未经确认不得提高限流追平5.5Nm。弹簧拟合及补偿残差同样待真实记录约束。

上机可用后，先由负责人决定构建/刷写/安全测试安排，并回传本次build和短冻结capture。当前结果只能说明主机工程和声明假设下的仿真有效，不能承诺修复后实机一定站稳。
