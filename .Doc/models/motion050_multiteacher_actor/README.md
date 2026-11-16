# Motion050 多Teacher蒸馏：015执行逻辑

分支 `feature/rl-motion31-supported050-multiteacher` 从远端015提交
`7f89e6d50e5060c23fc03fd78e9211dcd1c8823b` 创建。选用最终最佳050 Actor，
SHA256 `c21d66a2cd9af0b6d7452bc8dc4c32c3f13cc83d50173cfb14bfdab79546c87d`。
模型来源、ONNX、权重blob与合同身份见同目录 `manifest.json`。

## 实际替换范围

运行源码仅修改以下三个文件：

- `X-CUBE-AI/App/tjb_network_data_params.c`：49734个FP32参数、198936B，仅替换权重数组。
- `application/chassis/rl_motion31.h`：050动作合同、腿尺度0.50rad、有效高度0.20–0.40m及轮限矩4.8Nm。
- `application/chassis/rl_motion31.c`：原u2200几何＋11节点线性支撑中心；PD轮限矩使用4.8Nm。

输出为四腿 `center(height)+0.50*mean`，双轮 `80*mean`。
网络末层已有Tanh，不再Tanh、不累积腿步长、不使用Recovery三次decoder。
015原五次C0中心和050原u2200中心不同；只把0.15改成0.50会用错物理目标。
0.34m零mean四腿中心约为 `[-0.11784643,-0.15540549,+0.11784643,-0.15540549]rad`。
高度范围随模型合同扩展；原高度拨杆、直接请求和允许运动中变高的执行逻辑保留。
MuJoCo验收采用先变高再运动，不把原015的运动中连续变高直接宣称为已验证能力。

## 保留的015逻辑

`rl_control.c/h`、`rl_shadow.c/h`、`chassis.c`、命令、IMU、CAN、电机任务和调度不改。
原5秒腿长渐变、5秒接管等待、最新有效反馈/目标、100Hz Actor、200Hz PD保留。
不加入仿真的20ms观测、5ms目标、7/11ms轮力矩运输队列。
实际previous仍由原执行器提交；影子推理不写执行缓存。
31D列序、四腿q和previous腿÷1.5、previous轮÷100、真实dq×0.05保持。
模型left对应几何右，不交换网络通道，也不更改编码器零位。

腿PD40/2、轮Kv0.2、015实际弹簧补偿一次、最终腿限矩40Nm保持。
轮PD限矩由015的5.5Nm明确收敛到用户要求的4.8Nm，原始电流限幅16384及换算保持。
原控制退出、故障锁存、LQR回退均保持；此分支不合入Recovery2100会话或自动交接实现。
旧生成报告保留原X-CUBE-AI图生成身份；当前权重身份以本目录manifest和C数组SHA为准。

## 验证与边界

统一主机回归：`sh tests/run_rl_host_tests.sh`。
Python工具回归：`python -B -m unittest discover -s tools -p 'test_*.py' -v`。
需要numpy、onnx及主机C编译器。`tests/data/motion050_golden.json`固定16条真实状态，
包含站立、低高度、高高度与转弯，验证C权重回读推理、实际C decoder、输入与PD；
401个高度点独立核对中心。该fixture无需训练数据目录即可运行。
额外的ONNX检查读取manifest来源路径，迁移后可用 `MOTION050_SOURCE_ONNX`指定同SHA文件。
另一套25D network的历史ONNX缺失时，相关两项检查明确skip，不影响当前Motion golden。

原015 Makefile有Windows大小写假设。Linux构建使用命令行覆盖，Makefile保持原字节：

```sh
motion_headers=$(mktemp -d)
ln -s "$PWD/modules/motor/DMmotor/dmmotor.h" "$motion_headers/DMmotor.h"
C_INCLUDE_PATH="$motion_headers" CPLUS_INCLUDE_PATH="$motion_headers" \
make -j8 SHELL=/bin/sh GCC_PATH=/path/to/arm-none-eabi/bin \
  ASMM_SOURCES= \
  'ASM_SOURCES=startup_stm32h723xx.s Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s'
```

实际构建命令、日志及HEX/BIN放在数据根 `motion050_firmware_c21d66a2_20261116/`。
本轮8组C主机回归通过，Python工具12项通过／2项旧25D来源检查skip。
完整128条真实状态的C数组推理最大mean误差2.3842e-7，实际固件C的解码／PD／输入编码误差0；
401个高度点中心核对通过。ARM ELF/BIN内权重blob SHA与选中模型一致。
链接FLASH434716B；DTCMRAM129536/131072B，动态栈余量尚未实测。
逐文件身份、固件SHA与验证边界见同目录 `verification.json`。
ARM编译和主机数值核验不等于板端推理时延、供电、机械限位或实机闭环验收。
模型仅在固定vM1 MuJoCo下完成有限56场景×2seed复刻；实机仍为NOT_RUN。
