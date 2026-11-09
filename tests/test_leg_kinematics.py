"""Host regression: compile actual getPhi/spd; compare independent finite differences."""
import ctypes as c
import math
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
s = (root / 'application/balance/legalgorithm.c').read_text(encoding='utf-8-sig')
s = s[s.index('void getPhi('):s.index('LEGInstance *FK_Result_init')]
with tempfile.TemporaryDirectory() as tmp:
    p = Path(tmp)
    (p/'test.c').write_text('#include <math.h>\n#include <stdint.h>\n#define arm_sin_f32 sinf\n#define arm_cos_f32 cosf\nstatic float DWT_GetDeltaT(uint32_t *p){(void)p;return .002f;}\n'+s)
    subprocess.run(['cc','-shared','-fPIC','-O2',str(p/'test.c'),'-lm','-o',str(p/'test.so')],check=True)
    lib = c.CDLL(str(p/'test.so')); F = c.c_float; P = c.POINTER(F)
    lib.getPhi.argtypes = [F]*7+[P]*4
    lib.spd.argtypes = [F]*9+[P]*4
    def call(fn, args):
        v=[F() for _ in range(4)]
        fn(*args, *[c.byref(x) for x in v])
        return [x.value for x in v]
    count=0; maximum=0
    for lengths in [( .21,.25,.25,.21,0),(.205,.255,.245,.215,.025)]:
        for a in [1.7,1.9,2.1,2.3,2.5,2.7]:
            for b in [.4,.6,.8,1,1.2,1.4]:
                for da,db in [(1,0),(0,1),(1,1),(1,-1),(-.7,1.3)]:
                    h=.0005
                    hi=call(lib.getPhi,[a+h*da,b+h*db,*lengths])
                    lo=call(lib.getPhi,[a-h*da,b-h*db,*lengths])
                    got=call(lib.spd,[da,db,*lengths,a,b])
                    expected=[(hi[2]-lo[2])/(2*h),-math.remainder(hi[3]-lo[3],2*math.pi)/(2*h)]
                    err=max(abs(got[i]-expected[i]) for i in range(2))
                    assert math.isfinite(err) and err<.002,(lengths,a,b,da,db,got,expected)
                    if lengths[-1]==0:
                        assert abs(got[1]+.5*(da+db))<.0001
                    maximum=max(maximum,err);count+=1
    print(f'kinematics: {count} cases passed; max derivative error {maximum:.7g}')
