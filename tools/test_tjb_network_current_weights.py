"""Verify the checked-in C0 ONNX against firmware bytes and decoder.

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
MODEL = ROOT / '.Doc/models/motion_C0_standing_trial'
PARAMS = ROOT / 'X-CUBE-AI/App/tjb_network_data_params.c'


def firmware_blob():
    match = converter.array_pattern('tjb_network').search(PARAMS.read_text())
    words = re.findall(r'0x([0-9a-fA-F]{16})U', match.group(2))
    return b''.join(int(word, 16).to_bytes(8, 'little') for word in words)


class CurrentWeightsTests(unittest.TestCase):
    def test_model_declares_full_actor20_v4legs_and_replacement_wheel_delays(self):
        timing = json.loads((MODEL/'execution_timing.json').read_text())
        self.assertEqual(timing['actor_packet_age_ms'],20)
        self.assertEqual(timing['actor_physical_columns'],list(range(16)))
        self.assertTrue(timing['includes_four_leg_q_dq'])
        self.assertTrue(timing['complete_v4_leg_tuple'])
        self.assertEqual(timing['wheel_channel2_geometric_right_ms'],7)
        self.assertEqual(timing['wheel_channel5_geometric_left_ms'],11)
        self.assertEqual(timing['wheel_tau_ms'],0)
        self.assertFalse(timing['old_wheel20_tau2_retained'])
        metadata = {p.key:p.value for p in onnx.load(str(MODEL/'actor.onnx')).metadata_props}
        self.assertEqual(json.loads(metadata['output_contract']),json.loads((MODEL/'output_contract.json').read_text()))

    def test_onnx_bytes_and_provenance(self):
        blob, count = converter.extract_weights(onnx.load(str(MODEL/'actor.onnx')))
        self.assertEqual(count, 14630)
        self.assertEqual(blob, firmware_blob())
        manifest = json.loads((MODEL/'firmware_binding.json').read_text())
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
        self.assertEqual(declaration['action_version'], 'motion_multiheight_support_center_015_80_v1')
        with tempfile.TemporaryDirectory() as temporary:
            library = Path(temporary)/'decoder.so'
            subprocess.run(['cc','-std=c11','-O2','-shared','-fPIC',
                            str(ROOT/'application/chassis/rl_motion31.c'),'-lm','-o',str(library)],check=True)
            lib = ctypes.CDLL(str(library))
            ptr = np.ctypeslib.ndpointer(dtype=np.float32, ndim=1, flags='C_CONTIGUOUS')
            lib.RLMotion31DecodeOutput.argtypes = [ptr,ctypes.c_float,ptr]
            with np.load(MODEL/'physical_decode_golden.npz') as data:
                actions = data['mean_action']; heights=data['command_height_m']; expected=data['physical_target']
            for action,height,want in zip(actions,heights,expected):
                target = np.zeros(6,np.float32)
                self.assertEqual(lib.RLMotion31DecodeOutput(action,float(height),target),0)
                np.testing.assert_allclose(target,want,atol=8e-6,rtol=0)
            # Feed identical recorded physical feedback to the real firmware
            # PD function; this requested target is not an adopted-cache write.
            class Snapshot(ctypes.Structure):
                _fields_=[('gyro',ctypes.c_float*3),('gravity',ctypes.c_float*3),
                    ('q',ctypes.c_float*4),('dq',ctypes.c_float*6),('previous',ctypes.c_float*6),
                    ('valid',ctypes.c_uint8),('leg_tau',ctypes.c_float*4),('wheel_tau',ctypes.c_float*2),
                    ('tau_valid',ctypes.c_uint8),('vx',ctypes.c_float),('yaw',ctypes.c_float),('height',ctypes.c_float)]
            lib.RLMotion31ComputePdTorque.argtypes=[ctypes.POINTER(Snapshot),ptr,ptr,ptr]
            with np.load(MODEL/'real_state_physical_PD_golden.npz') as data:
                for q,dq,target,want in zip(data['PD_q'],data['PD_dq'],data['physical_target'],data['raw_PD_for_frozen_feedback']):
                    sample=Snapshot();sample.q[:]=q[[0,1,3,4]];sample.dq[:]=dq
                    got=np.zeros(6,np.float32);limited=got.copy()
                    self.assertEqual(lib.RLMotion31ComputePdTorque(ctypes.byref(sample),np.ascontiguousarray(target,np.float32),got,limited),0)
                    np.testing.assert_allclose(got,want,atol=5e-6,rtol=0)
            lib.RLMotion31SupportCenter.argtypes=[ctypes.c_double,np.ctypeslib.ndpointer(dtype=np.float64,ndim=1,flags='C_CONTIGUOUS')]
            with np.load(MODEL/'center_golden_vectors.npz') as data:
                for height,want in zip(data['command_height_m'],data['center_rad']):
                    got=np.zeros(4,np.float64)
                    self.assertEqual(lib.RLMotion31SupportCenter(float(height),got),0)
                    np.testing.assert_allclose(got,want,atol=1e-12,rtol=0)
            for height in [.29,.41,float('nan')]:
                self.assertNotEqual(lib.RLMotion31DecodeOutput(actions[0],height,np.zeros(6,np.float32)),0)
        print(f'Actual firmware decoder: {len(actions)} vectors match frozen 015/80 contract')


if __name__ == '__main__':
    unittest.main()
