# 更新 Motion31 固件权重

本脚本在不运行 CubeMX 的情况下，从 ONNX 更新 `X-CUBE-AI/App/tjb_network_data_params.c` 的权重数组。`tjb_network.c/.h` 及推理接口保持不变。

## 前提

- Python 3.9+；安装依赖：`python -m pip install onnx`
- 新 ONNX 必须是 `float32[1,31] -> float32[1,6]`，内部结构为 `31 -> 128 -> 64 -> 32 -> 6`，三层 ELU(alpha=1)，末层 Tanh。
- 四个 Gemm 的权重形状和转置参数必须与当前模型一致。输入输出维度相同但内部层数或宽度不同，脚本会拒绝。

## 使用

从 `combat-dm-balance-main` 目录执行：

```powershell
python tools/update_tjb_network_weights.py "D:\Combat\新模型.onnx" --dry-run
python tools/update_tjb_network_weights.py "D:\Combat\新模型.onnx"
```

第二条命令会修改 `X-CUBE-AI/App/tjb_network_data_params.c`。检查 `git diff -- X-CUBE-AI/App/tjb_network_data_params.c`，确认只有权重数组变化，再编译固件。已有模型生成文件有本地修改时，脚本不会重写其他文件。

脚本根据 ONNX Gemm 节点的权重和偏置张量，按 ST 当前布局排列为 58,520 字节的 FP32 数据，并转为 `ai_u64` 数组。它会核对模型拓扑、张量类型/尺寸、Gemm 与 ELU 属性和目标数组长度。若更换网络结构或输出语义，请重新用 ST Edge AI 生成整套 C 文件；此脚本不适用。
