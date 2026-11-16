# re-run the leg's joint test with the real pelvis plate (bigger than the test mount)
import sys; sys.path.insert(0, '/home/claude/leg'); sys.path.insert(0, '/home/claude/body')
import legmodel as L, body as B
orig = L.build_parts
def bp():
    P = orig()
    P['pelvis_test_mount'] = L.ring(L.R1) + [(1, L.wbox(B.X0, B.X1, -B.YW, B.YW, B.Z_PEL0, B.Z_PEL1))]
    return P
L.build_parts = bp
import os; os.chdir('/tmp'); exec(open('/home/claude/leg/rom.py').read())
