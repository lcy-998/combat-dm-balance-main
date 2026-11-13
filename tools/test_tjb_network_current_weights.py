"""Check the active tjb_network firmware weights against its source ONNX."""

import hashlib
import re
import unittest
from pathlib import Path

import numpy as np
import onnx
from onnx.reference import ReferenceEvaluator

import update_xcube_ai_weights as converter


ROOT = Path(__file__).resolve().parents[1]
MODEL = Path("D:/Combat/tjb_network/actor1112.onnx")
PARAMS = ROOT / "X-CUBE-AI/App/tjb_network_data_params.c"
REPORT = ROOT / "X-CUBE-AI/App/tjb_network_generate_report.txt"


def firmware_blob():
    match = converter.array_pattern("tjb_network").search(PARAMS.read_text(encoding="utf-8"))
    if match is None:
        raise AssertionError("tjb_network weight array is missing")
    words = re.findall(r"0x([0-9a-fA-F]{16})U", match.group(2))
    return b"".join(int(word, 16).to_bytes(8, "little") for word in words)


class CurrentWeightsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not MODEL.is_file():
            raise unittest.SkipTest(f"source ONNX is unavailable: {MODEL}")
        cls.model = onnx.load(str(MODEL))

    def test_onnx_matches_firmware_array(self):
        blob, count = converter.extract_weights(self.model)
        self.assertEqual(count, 49734)
        self.assertEqual(len(blob), 198936)
        self.assertEqual(blob, firmware_blob())
        self.assertIn("actor1112.onnx", REPORT.read_text(encoding="utf-8"))
        print(f"Current weights: {count} parameters, SHA256 {hashlib.sha256(blob).hexdigest()}")

    def test_firmware_weights_match_onnx_inference(self):
        weights = np.frombuffer(firmware_blob(), dtype="<f4")
        inputs = np.random.default_rng(1112).normal(size=(1, 31)).astype(np.float32)
        expected = ReferenceEvaluator(self.model).run(None, {self.model.graph.input[0].name: inputs})[0]
        actual = inputs.copy()
        offset = 0
        for layer, (nout, nin) in enumerate(converter.TARGETS["tjb_network"]["dims"]):
            matrix = weights[offset:offset + nout * nin].reshape(nout, nin)
            offset += nout * nin
            bias = weights[offset:offset + nout]
            offset += nout
            actual = actual @ matrix.T + bias
            if layer < 3:
                actual = np.where(actual < 0, np.expm1(actual), actual)
            else:
                actual = np.tanh(actual)
        self.assertEqual(offset, len(weights))
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-5)


if __name__ == "__main__":
    unittest.main()
