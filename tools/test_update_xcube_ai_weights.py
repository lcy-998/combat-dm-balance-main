import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import onnx

import update_xcube_ai_weights as converter


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / ".Doc/models/motion1600_C0_dual_teacher_actor/actor.onnx"
GENERATED = ROOT / "X-CUBE-AI" / "App" / "tjb_network_data_params.c"
NETWORK_MODEL = Path("D:/DefaultStore/QQ/mlp/RMUC_H723_smallMLP_v8_20261009/policy.onnx")
NETWORK_GENERATED = ROOT / "X-CUBE-AI" / "App" / "network_data_params.c"


class WeightConverterTests(unittest.TestCase):
    def test_matches_st_generated_bytes(self):
        for target, model_path, generated_path, count in (
            ("tjb_network", MODEL, GENERATED, 14630),
            ("network", NETWORK_MODEL, NETWORK_GENERATED, 20614),
        ):
            with self.subTest(target=target):
                if not model_path.is_file():
                    continue
                blob, actual_count = converter.extract_weights(onnx.load(str(model_path)), target)
                source = generated_path.read_text(encoding="utf-8")
                words = re.findall(r"0x([0-9a-fA-F]{16})U", converter.array_pattern(target).search(source).group(2))
                generated = b"".join(int(word, 16).to_bytes(8, "little") for word in words)
                self.assertEqual(actual_count, count)
                self.assertEqual(blob, generated)

    def test_rejects_wrong_target(self):
        if not NETWORK_MODEL.is_file():
            self.skipTest("original network ONNX is not available")
        with self.assertRaisesRegex(ValueError, "expected input shape"):
            converter.extract_weights(onnx.load(str(NETWORK_MODEL)), "tjb_network")

    def test_rejects_changed_gemm_layout(self):
        model = onnx.load(str(MODEL))
        next(attr for attr in model.graph.node[0].attribute if attr.name == "transB").i = 0
        with self.assertRaisesRegex(ValueError, "incompatible attributes"):
            converter.extract_weights(model)

    def test_rejects_changed_shape(self):
        model = onnx.load(str(MODEL))
        model.graph.input[0].type.tensor_type.shape.dim[1].dim_value = 30
        with self.assertRaisesRegex(ValueError, "expected input shape"):
            converter.extract_weights(model)

    def test_replaces_only_weight_array(self):
        model = onnx.load(str(MODEL))
        first = model.graph.initializer[0]
        data = bytearray(first.raw_data); data[0] ^= 1; first.raw_data = bytes(data)
        blob, _ = converter.extract_weights(model)
        with tempfile.TemporaryDirectory() as directory:
            replacement = Path(directory)/"changed.onnx"
            onnx.save(model,str(replacement))
            target = Path(directory) / "tjb_network_data_params.c"
            target.write_bytes(GENERATED.read_bytes())
            subprocess.run(
                [sys.executable, str(Path(converter.__file__)), str(replacement), "--output", str(target)],
                check=True,
                capture_output=True,
                text=True,
            )
            source = target.read_text(encoding="utf-8")
            match = converter.array_pattern("tjb_network").search(source)
            words = re.findall(r"0x([0-9a-fA-F]{16})U", match.group(2))
            actual = b"".join(int(word, 16).to_bytes(8, "little") for word in words)
            original = GENERATED.read_text(encoding="utf-8")
            self.assertEqual(actual, blob)
            original_match = converter.array_pattern("tjb_network").search(original)
            self.assertEqual(source[:match.start(2)], original[:original_match.start(2)])
            self.assertEqual(source[match.end(2):], original[original_match.end(2):])

    def test_network_replaces_only_its_weight_array(self):
        if not NETWORK_MODEL.is_file():
            self.skipTest("original network ONNX is not available")
        model = onnx.load(str(NETWORK_MODEL))
        first = model.graph.initializer[0]
        data = bytearray(first.raw_data)
        data[0] ^= 1
        first.raw_data = bytes(data)
        blob, _ = converter.extract_weights(model, "network")
        with tempfile.TemporaryDirectory() as directory:
            model_path = Path(directory) / "changed.onnx"
            target = Path(directory) / "network_data_params.c"
            onnx.save(model, str(model_path))
            target.write_bytes(NETWORK_GENERATED.read_bytes())
            subprocess.run(
                [sys.executable, str(Path(converter.__file__)), str(model_path),
                 "--target", "network", "--output", str(target)],
                check=True, capture_output=True, text=True,
            )
            source = target.read_text(encoding="utf-8")
            original = NETWORK_GENERATED.read_text(encoding="utf-8")
            pattern = converter.array_pattern("network")
            match, original_match = pattern.search(source), pattern.search(original)
            words = re.findall(r"0x([0-9a-fA-F]{16})U", match.group(2))
            actual = b"".join(int(word, 16).to_bytes(8, "little") for word in words)
            self.assertEqual(actual, blob)
            self.assertEqual(source[:match.start(2)], original[:original_match.start(2)])
            self.assertEqual(source[match.end(2):], original[original_match.end(2):])


if __name__ == "__main__":
    unittest.main()
