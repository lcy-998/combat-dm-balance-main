"""CPU-only continuous C0 center fit. Source knots remain read-only evidence."""
import argparse
import json
from pathlib import Path
import numpy as np


def fit_contract(source):
    h=np.array(source['support_offset_table']['height_m'])
    offsets=np.array(source['support_offset_table']['offset_rad'])
    x=(h-.35)/.05
    coefficients=np.polynomial.polynomial.polyfit(x,offsets,5).T
    candidates=list(x)
    for channel,coef in enumerate(coefficients):
        derivative=np.polynomial.polynomial.polyder(coef)
        for i in range(len(h)-1):
            d=derivative.copy();d[0]-=(offsets[i+1,channel]-offsets[i,channel])/(x[i+1]-x[i])
            candidates.extend(float(root.real) for root in np.polynomial.polynomial.polyroots(d)
                              if abs(root.imag)<1e-10 and x[i]<root.real<x[i+1])
    grid=np.array(sorted(set(candidates)));hg=.35+.05*grid
    polynomial=np.column_stack([np.polynomial.polynomial.polyval(grid,c) for c in coefficients])
    old=np.column_stack([np.interp(hg,h,offsets[:,i]) for i in range(2)])
    error=abs(polynomial-old);peak=error.argmax(0)
    return dict(kind='global_quintic_support_offset_v1',normalization=dict(height_origin_m=.35,height_scale_m=.05),
        coefficients_low_to_high=dict(hip=coefficients[0].tolist(),active=coefficients[1].tolist()),
        source_height_m=h.tolist(),anchor_max_error_rad=float(abs(np.column_stack([np.polynomial.polynomial.polyval(x,c) for c in coefficients])-offsets).max()),
        interval_extrema_max_difference_from_source_rad=error.max(0).tolist(),
        interval_extrema_height_m=hg[peak].tolist(),Kp40_max_request_difference_Nm=(40*error.max(0)).tolist(),
        difference_scope='All source intervals: polynomial-minus-linear extrema from derivative roots and endpoints; not only a grid scan',
        runtime_lookup=False,stationarity_gate=False,confirmation_timer=False,new_slew_limit=False,
        physical_transition_validation='NOT_RUN',encoder_delta='UNKNOWN_NOT_FIT_FROM_CENTER')


def support_center(h,contract):
    h=np.asarray(h,dtype=float);p=contract['support_offset_polynomial'];n=p['normalization']
    x=(h-n['height_origin_m'])/n['height_scale_m'];c=p['coefficients_low_to_high']
    hip_offset=np.polynomial.polynomial.polyval(x,c['hip'])
    active_offset=np.polynomial.polynomial.polyval(x,c['active'])
    f=contract['center_formula'];L=h+f['hip_above_root_m']-f['wheel_radius_m']
    beta=np.arccos((L*L+f['equivalent_upper_m']**2-f['equivalent_lower_m']**2)/(2*L*f['equivalent_upper_m']))
    hip=beta-np.deg2rad(f['hip_zero_deg'])+hip_offset
    active=beta-np.deg2rad(f['active_zero_deg'])+active_offset
    return np.stack([hip,active,-hip,active],axis=-1)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source_contract',type=Path)
    args=parser.parse_args()
    print(json.dumps(fit_contract(json.loads(args.source_contract.read_text())),indent=2))
