import numpy as np, csg
import legmodel as L
from build import part_solid, place
sv = L.P1
Rt = sv.R.T
def to_servo(s, perm=None):
    def f(v):
        q = Rt @ (np.array(v) - sv.P)          # servo frame (x_s, y_s, z_s)
        if perm == 'bridge_down': q = np.array([q[1], q[2], q[0]])    # bridge on the bed, plates standing up
        return tuple(q)
    return s.transform(f)
ring = to_servo(part_solid(L.ring(sv)))
# small base under the ring so it stands up and you can hold it in a vise/hand
br = to_servo(part_solid(L.bracket(sv, (-1, 0))), 'bridge_down')
for name, s in (('fit_ring', ring), ('fit_bracket', br)):
    csg.write_stl_clean(place(s), f'stl/{name}.stl', name)
    b = place(s).bbox(); print(name, 'size', round(b[1]-b[0],1), round(b[3]-b[2],1), round(b[5]-b[4],1))

