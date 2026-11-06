"""Verify the checked-in Motion2320 ONNX against firmware bytes and decoder.

Run with the same Python environment as update_xcube_ai_weights.py.
No ST runtime, hardware access or firmware flash is involved.
"""
import ctypes
import hashlib
import json
import re
import subprocess
import tempfile
import unittest
from pathlib import Path

import numpy as np
import onnx

import update_xcube_ai_weights as converter

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / '.Doc/models/motion2320_small31_delayed'
PARAMS = ROOT / 'X-CUBE-AI/App/tjb_network_data_params.c'


def firmware_blob():
    match = converter.array_pattern('tjb_network').search(PARAMS.read_text())
    words = re.findall(r'0x([0-9a-fA-F]{16})U', match.group(2))
    return b''.join(int(word, 16).to_bytes(8, 'little') for word in words)


class CurrentWeightsTests(unittest.TestCase):
    def test_onnx_bytes_and_provenance(self):
        blob, count = converter.extract_weights(onnx.load(str(MODEL/'actor.onnx')))
        self.assertEqual(count, 14630)
        self.assertEqual(blob, firmware_blob())
        manifest = json.loads((MODEL/'manifest.json').read_text())
        self.assertEqual(hashlib.sha256(blob).hexdigest(), manifest['firmware_weight_blob_sha256'])
        self.assertEqual(hashlib.sha256((MODEL/'actor.onnx').read_bytes()).hexdigest(), manifest['actor_onnx_sha256'])

    def test_firmware_blob_inference_on_golden_vectors(self):
        # Decode exactly the array compiled into firmware, independently of ONNX.
        weights = np.frombuffer(firmware_blob(), dtype='<f4')
        with np.load(MODEL/'golden_vectors.npz') as data:
            x, expected = data['observation'].copy(), data['mean_action']
        offset = 0
        for layer, (nout, nin) in enumerate(converter.TARGETS['tjb_network']['dims']):
            w = weights[offset:offset+nout*nin].reshape(nout, nin)
            offset += nout*nin
            b = weights[offset:offset+nout]
            offset += nout
            x = x @ w.T + b
            if layer < 3:
                negative = x < 0
                x[negative] = np.expm1(x[negative])
            else:
                x = np.tanh(x)
        error = float(np.max(np.abs(x-expected)))
        self.assertLess(error, 1e-5)
        print(f'Firmware array: {len(expected)} golden vectors, max error {error:.9g}')

    def test_real_firmware_decoder_matches_frozen_output_contract(self):
        declaration = json.loads((MODEL/'output_contract.json').read_text())
        self.assertEqual(declaration['action_version'], 'motion31_kinematic_support_centered_015_80_v1')
        formula = declaration['center_formula']
        length = .34 + formula['hip_above_root_m'] - formula['wheel_radius_m']
        upper, lower = formula['equivalent_upper_m'], formula['equivalent_lower_m']
        beta = np.arccos((length**2+upper**2-lower**2)/(2*length*upper))
        hip = beta - np.deg2rad(formula['hip_zero_deg'])
        active = beta - np.deg2rad(formula['active_zero_deg'])
        center = np.zeros(6)
        center[[0,1,3,4]] = np.array([hip,active,-hip,active])+declaration['support_target_offset_rad']
        scale = np.array([.15,.15,80.,.15,.15,80.])
        with tempfile.TemporaryDirectory() as temporary:
            library = Path(temporary)/'decoder.so'
            subprocess.run(['cc','-std=c11','-O2','-shared','-fPIC',
                            str(ROOT/'application/chassis/rl_motion31.c'),'-lm','-o',str(library)],check=True)
            lib = ctypes.CDLL(str(library))
            ptr = np.ctypeslib.ndpointer(dtype=np.float32, ndim=1, flags='C_CONTIGUOUS')
            lib.RLMotion31DecodeOutput.argtypes = [ptr,ctypes.c_float,ptr]
            with np.load(MODEL/'golden_vectors.npz') as data:
                actions = data['mean_action']
            for action in actions:
                target = np.zeros(6,np.float32)
                self.assertEqual(lib.RLMotion31DecodeOutput(action,.34,target),0)
                expected = center+scale*np.clip(action,-1,1)
                np.testing.assert_allclose(target,expected,atol=8e-6,rtol=0)
        print(f'Actual firmware decoder: {len(actions)} vectors match frozen 015/80 contract')


if __name__ == '__main__':
    unittest.main()
