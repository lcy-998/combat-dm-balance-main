"""Check the active tjb_network firmware weights against its source ONNX."""

import ctypes as ct
import hashlib
import json
import math
import os
import re
import subprocess
import tempfile
import unittest
from pathlib import Path

import numpy as np
import onnx
from onnx.reference import ReferenceEvaluator

import update_xcube_ai_weights as converter


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = json.loads(
    (ROOT / ".Doc/models/motion050_multiteacher_actor/manifest.json").read_text()
)
MODEL = Path(os.environ.get("MOTION050_SOURCE_ONNX", MANIFEST["source_onnx"]["path"]))
PARAMS = ROOT / "X-CUBE-AI/App/tjb_network_data_params.c"


def firmware_blob():
    match = converter.array_pattern("tjb_network").search(
        PARAMS.read_text(encoding="utf-8")
    )
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
        self.assertEqual(
            hashlib.sha256(MODEL.read_bytes()).hexdigest(),
            MANIFEST["source_onnx"]["sha256"],
        )
        print(
            f"Current weights: {count} parameters, SHA256 {hashlib.sha256(blob).hexdigest()}"
        )

    def test_firmware_weights_match_onnx_inference(self):
        weights = np.frombuffer(firmware_blob(), dtype="<f4")
        inputs = np.random.default_rng(1112).normal(size=(1, 31)).astype(np.float32)
        expected = ReferenceEvaluator(self.model).run(
            None, {self.model.graph.input[0].name: inputs}
        )[0]
        actual = inputs.copy()
        offset = 0
        for layer, (nout, nin) in enumerate(converter.TARGETS["tjb_network"]["dims"]):
            matrix = weights[offset : offset + nout * nin].reshape(nout, nin)
            offset += nout * nin
            bias = weights[offset : offset + nout]
            offset += nout
            actual = actual @ matrix.T + bias
            if layer < 3:
                actual = np.where(actual < 0, np.expm1(actual), actual)
            else:
                actual = np.tanh(actual)
        self.assertEqual(offset, len(weights))
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-5)


class Snapshot(ct.Structure):
    _fields_ = [
        ("gyro", ct.c_float * 3),
        ("gravity", ct.c_float * 3),
        ("q", ct.c_float * 4),
        ("dq", ct.c_float * 6),
        ("previous", ct.c_float * 6),
        ("previous_valid", ct.c_uint8),
        ("lqr_legs", ct.c_float * 4),
        ("lqr_wheels", ct.c_float * 2),
        ("lqr_valid", ct.c_uint8),
        ("vx", ct.c_float),
        ("yaw", ct.c_float),
        ("height", ct.c_float),
    ]


class RealStateGoldenTests(unittest.TestCase):
    """The committed real-state fixture works without external model files."""

    @classmethod
    def setUpClass(cls):
        fixture = ROOT / MANIFEST["golden_fixture"]["path"]
        assert (
            hashlib.sha256(fixture.read_bytes()).hexdigest()
            == MANIFEST["golden_fixture"]["sha256"]
        )
        cls.rows = json.loads(fixture.read_text())["states"]
        cls.weights = np.frombuffer(firmware_blob(), dtype="<f4")
        directory = tempfile.TemporaryDirectory(prefix="motion050-golden-")
        cls.addClassCleanup(directory.cleanup)
        library = Path(directory.name) / "motion050.so"
        subprocess.run(
            [
                os.environ.get("CC", "cc"),
                "-shared",
                "-fPIC",
                "-std=c11",
                "-O2",
                "-Wall",
                "-Wextra",
                "-Werror",
                str(ROOT / "application/chassis/rl_motion31.c"),
                "-lm",
                "-o",
                str(library),
            ],
            check=True,
        )
        cls.core = ct.CDLL(str(library))
        pointer = ct.POINTER(ct.c_float)
        cls.core.RLMotion31SupportCenter.argtypes = [
            ct.c_double,
            ct.POINTER(ct.c_double),
        ]
        cls.core.RLMotion31DecodeOutput.argtypes = [pointer, ct.c_float, pointer]
        cls.core.RLMotion31ComputePdTorque.argtypes = [
            ct.POINTER(Snapshot),
            pointer,
            pointer,
            pointer,
        ]
        cls.core.RLMotion31BuildObservation.argtypes = [ct.POINTER(Snapshot), pointer]

    def test_firmware_weight_identity_and_real_state_means(self):
        self.assertEqual(len(self.weights), 49734)
        self.assertEqual(
            hashlib.sha256(firmware_blob()).hexdigest(),
            MANIFEST["firmware_weights"]["blob_sha256"],
        )
        maximum = 0.0
        for row in self.rows:
            actual = np.asarray([row["observation"]], dtype=np.float32)
            offset = 0
            for layer, (nout, nin) in enumerate(
                converter.TARGETS["tjb_network"]["dims"]
            ):
                matrix = self.weights[offset : offset + nout * nin].reshape(nout, nin)
                offset += nout * nin
                bias = self.weights[offset : offset + nout]
                offset += nout
                actual = actual @ matrix.T + bias
                actual = (
                    np.where(actual < 0, np.expm1(actual), actual)
                    if layer < 3
                    else np.tanh(actual)
                )
            self.assertEqual(offset, len(self.weights))
            np.testing.assert_allclose(actual[0], row["mean"], rtol=0, atol=1e-6)
            maximum = max(maximum, float(np.max(np.abs(actual[0] - row["mean"]))))
        print(
            f"C-array inference: {len(self.rows)} real single inputs, max error {maximum:.9g}"
        )

    def test_real_firmware_decoder_PD_and_input_encoding(self):
        maximum = np.zeros(4)
        for row in self.rows:
            observation = np.asarray(row["observation"], dtype=np.float32)
            mean = (ct.c_float * 6)(*row["mean"])
            target, raw, limited = [(ct.c_float * 6)() for _ in range(3)]
            sample = Snapshot()
            sample.q[:], sample.dq[:] = row["q"], row["dq"]
            sample.height = float(observation[24] * 0.5)
            self.assertEqual(
                self.core.RLMotion31DecodeOutput(mean, sample.height, target), 0
            )
            np.testing.assert_allclose(target, row["target"], rtol=0, atol=2e-6)
            self.assertEqual(
                self.core.RLMotion31ComputePdTorque(
                    ct.byref(sample), target, raw, limited
                ),
                0,
            )
            np.testing.assert_allclose(raw, row["pd_nm"], rtol=0, atol=2e-5)
            np.testing.assert_allclose(limited, row["limited_nm"], rtol=0, atol=2e-5)
            # Actor and current PD feedback are different timestamps. Re-encode
            # the Actor packet; retain the newer real feedback above for PD.
            sample.q[:] = observation[6:10] * 1.5
            sample.dq[:] = observation[10:16] / 0.05
            sample.gyro[:], sample.gravity[:] = observation[:3] / 0.25, observation[3:6]
            sample.previous[:], sample.previous_valid = row["previous"], 1
            sample.vx, sample.yaw = (
                float(observation[22] * 4),
                float(observation[23] * 4),
            )
            encoded = (ct.c_float * 31)()
            self.assertEqual(
                self.core.RLMotion31BuildObservation(ct.byref(sample), encoded), 0
            )
            np.testing.assert_allclose(encoded, observation, rtol=0, atol=1e-6)
            pairs = [
                (target, row["target"]),
                (raw, row["pd_nm"]),
                (limited, row["limited_nm"]),
                (encoded, observation),
            ]
            maximum = np.maximum(
                maximum, [np.max(np.abs(np.asarray(a) - b)) for a, b in pairs]
            )
        print(
            f"Firmware golden max errors [decode, PD, limited, input]: {maximum.tolist()}"
        )

    def test_centers_match_frozen_contract_at_nodes_and_between_nodes(self):
        contract = MANIFEST["output_contract"]
        geometry, table = contract["center_formula"], contract["support_offset_table"]
        for height in np.linspace(0.2, 0.4, 401):
            center = (ct.c_double * 4)()
            self.assertEqual(self.core.RLMotion31SupportCenter(height, center), 0)
            length = height + geometry["hip_above_root_m"] - geometry["wheel_radius_m"]
            upper, lower = (
                geometry["equivalent_upper_m"],
                geometry["equivalent_lower_m"],
            )
            beta = math.acos(
                (length * length + upper * upper - lower * lower) / (2 * length * upper)
            )
            offsets = np.array(table["offset_rad"])
            hip = (
                beta
                - math.radians(geometry["hip_zero_deg"])
                + np.interp(height, table["height_m"], offsets[:, 0])
            )
            active = (
                beta
                - math.radians(geometry["active_zero_deg"])
                + np.interp(height, table["height_m"], offsets[:, 1])
            )
            np.testing.assert_allclose(
                center, [hip, active, -hip, active], rtol=0, atol=1e-12
            )


if __name__ == "__main__":
    unittest.main()
