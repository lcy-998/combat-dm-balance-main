# 更新 X-CUBE-AI 固件权重

本脚本从 ONNX 选择性更新 `tjb_network` 或 `network` 的权重数组，无需运行 CubeMX。只修改选中目标的 `X-CUBE-AI/App/<目标>_data_params.c`；推理代码和接口保持不变。

## 前提

- Python 3.9+；安装依赖：`python -m pip install onnx`
- Windows 上若 `python` 指向其他解释器，请使用安装了 `onnx` 的 Python 完整路径运行脚本。例如本机可将下方命令的 `python` 替换为 `& 'C:\Users\18167\AppData\Local\Programs\Python\Python313\python.exe'`。用同一个解释器执行 `-m pip install onnx`。
- `tjb_network`：输入 `float32[1,31]`，网络 `31 -> 128 -> 64 -> 32 -> 6`，三层 ELU(alpha=1)，末层 Tanh；权重 58,520 字节。
- `network`：输入 `float32[1,25]`，网络 `25 -> 128 -> 128 -> 6`，两层 ReLU，最后一层直接输出；权重 82,456 字节。
- 两者输出均为 `float32[1,6]`。Gemm 权重形状和转置参数必须与当前模型一致。只允许更改权重和偏置数值。

## 使用

从 `combat-dm-balance-main` 目录执行：

```powershell
python tools/update_xcube_ai_weights.py "D:\Combat\新actor.onnx" --target tjb_network --dry-run
python tools/update_xcube_ai_weights.py "D:\Combat\新actor.onnx" --target tjb_network

python tools/update_xcube_ai_weights.py "D:\Combat\新policy.onnx" --target network --dry-run
python tools/update_xcube_ai_weights.py "D:\Combat\新policy.onnx" --target network
```

省略 `--target` 时默认为 `tjb_network`。检查 `git diff -- X-CUBE-AI/App/<目标>_data_params.c`，确认只有权重数组变化，再编译固件。已有模型生成文件有本地修改时，脚本不会重写其他文件。

脚本根据 ONNX Gemm 节点的权重和偏置张量，按所选目标的 ST 布局排列 FP32 数据，并转为 `ai_u64` 数组。它会核对模型拓扑、张量类型/尺寸、Gemm 与 ELU 属性和目标数组长度。若更换网络结构或输出语义，请重新用 ST Edge AI 生成整套 C 文件；此脚本不适用。

## 015分支当前模型

`feature/rl-motion31-supported015`使用1600＋C0双教师蒸馏的单个小Actor。当前来源包与固件连续高度公式的绑定见[接入记录](../.Doc/models/motion1600_C0_dual_teacher_actor/FIRMWARE_INTEGRATION.md)。源ONNX保留训练查表合同；活动固件继续使用既有poly5合同，两者在已验证的0.34m锚点一致。更新器仅检查网络数值布局，不能替代物理合同核验。

```sh
python tools/update_xcube_ai_weights.py .Doc/models/motion1600_C0_dual_teacher_actor/actor.onnx --target tjb_network --dry-run
python tools/update_xcube_ai_weights.py .Doc/models/motion1600_C0_dual_teacher_actor/actor.onnx --target tjb_network
python tools/test_update_xcube_ai_weights.py
python tools/test_tjb_network_current_weights.py
```

当前网络测试绑定此模型，核对固件数组、1,488组Actor/物理目标/PD数据及既有连续中心公式。原C0模型包保持为历史证据。
