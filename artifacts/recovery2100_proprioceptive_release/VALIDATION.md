# 本体感知自动交接收据

默认流程、代码入口和操作见docs/RECOVERY2100_HANDOFF.md，烧写产物及SHA见firmware/recovery2100_motion015/manifest.json。

- 电控原8组+Recovery/交接回归9/9；真实策略任务14情景；新增本体姿态/构型阈值、NaN/倒立/高速拒绝、反馈重复不计dwell、时间wrap及真实Motion握手通过。
- Python工具集9项无失败，其中6项因未提供外部原ONNX跳过；015模型生成文件未变。
- ARM完整构建通过：Flash639900B，DTCM82592B，RAM_D1 capture59456B。静态内存证明不替代目标MCUdeadline/高水位，目标实测尚未执行。
- 仿真7条件：6条件0.60–0.94s自动切换、保持12.06–12.40s；共同失败条件在切换前。实际157180物理子步/90.1097s。没有训练新模型。
- 仿真仓正式portable入口scripts/test_portable_cpu.py：2011passed/31skipped/14warnings，架构217/0，Ruff0.8.4通过。曾误用pytest -m portable触发不应收集的旧实验缺文件，改按既有正式portable入口执行，未创建旧实验或修改测试以掩盖失败。
- 本体感知交接不读取外部定位、法向力常数或仿真高度。严格支持力API保留为另一明确来源；来源变化重置dwell。
- 未刷机，未声称实机可任意倒地起身，未认证触地。

用户追加尽早交接要求，最终固件本体门100ms。posture100ms_r1再次7组：6组0.50–0.85s切换后保持12秒以上，1组交接前失败；实际157180子步/92.8567s。对应fast100ms_*.json与*_fast100ms.log，最终Flash639932B。原200ms证据保留，不混同为最终阈值。
