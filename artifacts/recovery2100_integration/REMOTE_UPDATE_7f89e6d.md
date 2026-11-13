# 远端015更新核验

已fetch并合入远端015提交`7f89e6d`，merge为`6af87e0`。原VALIDATION/source_receipt为此前3b448f4交付时的历史快照，本记录为更新后增量验证。

`tools/update_xcube_ai_weights.py --target tjb_network`已支持31→256→128→64→6、ELU(alpha1)/Tanh、49734个FP32参数、198936 B。只更新ST ai_u64权重数组，不更改decoder/输入归一化/控制合同。

当前固件权重重建成标准Gemm/ELU/Tanh ONNX，再经转换器提取和--dry-run逐字节回环一致，输出unchanged/no file written。一次性重建脚本在/tmp/verify_015_converter.py，原始actor1112.onnx不在本机，此回环不是原始ONNX来源认证。

工具集9测试通过，6项因外部原始ONNX不可用跳过；电控9/9主机测试通过，含真实shadow14情景；ARM完整构建通过，Flash638812 B、DTCM82560 B。最新调试字段frame_begin改publish_seq，Recovery新增测试已同步。

当前Recovery部署包的actor.onnx含动态batch及物理解码，不能直接交此mean-only固定[1,31]转换器；实际dry-run在输入形状边界拒绝。现有Recovery独立C网络接入保持，未将其写进Motion权重。需要转换Recovery时应先导出固定batch的纯mean网络，物理解码仍由Recovery合同执行一次。

未改动015生成网络、未刷机、未推送。旧权重工具不适配256的问题已由此次远端更新修复；旧失败日志按原版本身份保留。
