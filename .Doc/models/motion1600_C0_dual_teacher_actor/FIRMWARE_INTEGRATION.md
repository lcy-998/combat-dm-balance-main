# 1600＋C0双教师小Actor：015固件接入

当前网络为单个31→128→64→32→6、ELU/Tanh Actor，14,630参数、58,520字节权重。训练以1600为主教师，C0补充站立与部分接扰。部署不运行教师或选择器。

通过`tools/update_xcube_ai_weights.py --target tjb_network`更新固件数组。`student.pt`与`actor.onnx`逐字节保留源包，SHA见`SHA256SUMS`和`firmware_binding.json`。原始README、manifest、verification描述导出时状态；本次固件集成状态以`firmware_binding.json`为准。

## 来源合同与活动合同

- `output_contract.json`及ONNX内嵌合同为训练时的查表中心，版本`motion_multiheight_support_center_015_80_v1`；原始`support_center.*`、`center_golden_vectors.npz`属于此合同。
- `firmware_output_contract.json`为015分支现有连续五次高度公式，版本`motion_multiheight_support_center_poly5_015_80_v1`，与上一提交完全相同。`firmware_center_fit.json`和`firmware_center_golden_vectors.npz`是该公式的CPU数值参考。当前执行源码为`application/chassis/rl_motion31.c`。
- 两者六个锚点一致，0.34m四腿中心为[-0.13097405434,-0.15117678046,+0.13097405434,-0.15117678046]rad。锚点间最大差约髋0.003870rad、主动0.001535rad；精确极值见拟合文件。新学生只在固定0.34m验证，不能据此宣称动态高度或其他高度闭环通过。

输入31D保持腿q与实际采用腿目标÷1.5、实际采用轮目标÷100、真实六轴dq×.05；输出腿中心±.15rad、轮±80rad/s。模型left对应几何右，不重排六轴，不重复Tanh。previous仍由本地owner采用绝对物理目标后更新。

保留原0.30–0.40m命令定义域、动态高度流程、在途目标原高度解码和实际采用previous语义。保持零位、方向、PD40/2/.2、限矩、电流限幅及10ms Actor/5ms owner。实机使用最新有效反馈，不人为添加仿真延迟。encoder delta仍UNKNOWN。

## 已有MuJoCo能力证据

源结果为0.34m、名义完整V4、Actor物理字段20ms、目标5ms、轮ch2/ch5为7/11ms且tau0；物理/PD/策略1/5/10ms。完整DR和动态高度未验证。详情见`capability_evidence.json`、`final_summary.json`和`distillation_report.md`，这些是训练run的冻结证据，本次集成未新增物理步或优化。

两个新seed共62回合，学生62回合无跌倒、50项满足完整任务门；1600为60回合无跌倒、46项满足任务门。1600已通过的任务均保留。±3m/s直线、腿扰16/16、机体推扰16/16通过。站立腿10–40Hz速度RMS较1600下降59.4%，但后漂约0.203m/s。±4/±8rad/s原地转向未满足双轮同时承重90%门限，不能认定全任务通过。

## CPU复核

用安装了onnx与numpy的同一Python解释器执行：

```sh
python tools/update_xcube_ai_weights.py .Doc/models/motion1600_C0_dual_teacher_actor/actor.onnx --target tjb_network --dry-run
python tools/test_update_xcube_ai_weights.py
python tools/test_tjb_network_current_weights.py
sh tests/run_rl_host_tests.sh
CC=clang sh tests/run_rl_host_tests.sh
```

复核固件数组与源ONNX逐字节相等、1,488组Actor golden、真实固件解码/PD与连续中心参考，以及控制回归。数值结果在`firmware_verification.json`；集成文件校验见`FIRMWARE_SHA256SUMS`，原始源包校验仍见`SHA256SUMS`。

本次未刷板；主机未提供ARM GCC，未做整板链接、板端ST runtime或实机闭环验证。
