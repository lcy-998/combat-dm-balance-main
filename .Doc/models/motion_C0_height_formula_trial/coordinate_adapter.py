"""Pure MCU/model coordinate boundary; no IO, estimation, trim or state cache.

Angles are already in the unified model axes and radians. Leg order is
geometric right hip/active, left hip/active (six-channel indices0/1/3/4).
delta := q_report - q_model is a constant, externally measured calibration.
None means UNKNOWN and is never interpreted as zero. Wheel commands are
velocities and never receive a position offset.
"""
from __future__ import annotations
import numpy as np

LEGS = [0, 1, 3, 4]


def _finite(value, width):
    x = np.asarray(value, dtype=float)
    if x.ndim < 1 or x.shape[-1] != width or not np.isfinite(x).all():
        raise ValueError(f'Finite final dimension{width} required')
    return x


def require_delta(delta_rad):
    if delta_rad is None:
        raise ValueError('Encoder delta UNKNOWN; external geometric calibration required')
    d = _finite(delta_rad, 4)
    if d.shape != (4,):
        raise ValueError('One constant four-leg delta required; no online per-frame trim')
    return d


def q_model_from_report(q_report_rad, delta_rad):
    return _finite(q_report_rad, 4) - require_delta(delta_rad)


def q_report_from_model(q_model_rad, delta_rad):
    return _finite(q_model_rad, 4) + require_delta(delta_rad)


def observation_model_from_report(observation_report, delta_rad):
    """31D report-coordinate encoding -> the original absolute model encoding.

    Input columns6:10 contain q_report/1.5;16:20 contain the actual adopted
    previous_report/1.5. This is not a center-relative observation. All dq,
    gyro, gravity, wheels, commands and one-hot fields are copied unchanged.
    """
    x = _finite(observation_report, 31).copy()
    d = require_delta(delta_rad) / 1.5
    x[..., 6:10] -= d
    x[..., 16:20] -= d
    return x


def target_report_from_model(target_model, delta_rad):
    x = _finite(target_model, 6).copy()
    x[..., LEGS] += require_delta(delta_rad)
    return x


def target_model_from_report(target_report, delta_rad):
    x = _finite(target_report, 6).copy()
    x[..., LEGS] -= require_delta(delta_rad)
    return x


def limits_model_from_report(leg_limits_report_rad, delta_rad):
    x = np.asarray(leg_limits_report_rad, dtype=float)
    if x.shape != (4, 2) or not np.isfinite(x).all() or (x[:, 0] > x[:, 1]).any():
        raise ValueError('Four finite ordered position limits required')
    return x - require_delta(delta_rad)[:, None]


def center_from_unprotected_request(mean, target_model, *, request_layer, mechanical_clipped):
    """Algebraic candidate only: c = requested_q_model - .15*mean_leg.

    Never use measured q as a PD request, unaligned transported targets or a
    mechanically clipped/adopted target to infer c. This does not identify
    encoder delta and does not modify a frozen policy/center configuration.
    """
    m, target = _finite(mean, 6), _finite(target_model, 6)
    if m.shape != target.shape or (abs(m) > 1 + 1e-6).any():
        raise ValueError('Matching bounded six-channel mean and physical request required')
    if request_layer != 'unprotected_request' or np.asarray(mechanical_clipped).any():
        raise ValueError('Cannot infer center from protected/adopted or unidentified targets')
    return target[..., LEGS] - .15 * m[..., LEGS]
