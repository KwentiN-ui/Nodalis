#!/usr/bin/env python3
"""Writes a CalculiX-style input of a hexahedral block for solver benchmarks.

    gen_block.py <nx> <ny> <nz> <lx> <ly> <lz> [C3D8|C3D20R|C3D4|C3D10] [direct|iterative] > job.inp

The tetrahedral kinds split every hexahedron into six tetrahedra around its diagonal.

The block is clamped at x = 0 and loaded by a pressure on the top face (z = lz). Thin,
long blocks (lx >> lz) are badly conditioned and show what a preconditioner is worth.
"""
import sys

nx, ny, nz = (int(a) for a in sys.argv[1:4])
lx, ly, lz = (float(a) for a in sys.argv[4:7])
kind = sys.argv[7] if len(sys.argv) > 7 else "C3D8"
solver = sys.argv[8] if len(sys.argv) > 8 else "direct"
out = sys.stdout

def nid(i, j, k):
    return 1 + i + (nx + 1) * (j + (ny + 1) * k)

out.write("*NODE, NSET=NALL\n")
for k in range(nz + 1):
    for j in range(ny + 1):
        for i in range(nx + 1):
            out.write(f"{nid(i, j, k)}, {lx * i / nx:.6g}, {ly * j / ny:.6g}, {lz * k / nz:.6g}\n")
mid = {}
edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
         (0, 4), (1, 5), (2, 6), (3, 7)]

def corners(i, j, k):
    return [(i, j, k), (i + 1, j, k), (i + 1, j + 1, k), (i, j + 1, k),
            (i, j, k + 1), (i + 1, j, k + 1), (i + 1, j + 1, k + 1), (i, j + 1, k + 1)]

if kind == "C3D20R":
    # Edge midside nodes, numbered after the corners.
    n = (nx + 1) * (ny + 1) * (nz + 1)
    for k in range(nz):
        for j in range(ny):
            for i in range(nx):
                c = corners(i, j, k)
                for a, b in edges:
                    key = (min(c[a], c[b]), max(c[a], c[b]))
                    if key not in mid:
                        n += 1
                        mid[key] = n
                        (ia, ja, ka), (ib, jb, kb) = key
                        out.write(f"{n}, {lx * (ia + ib) / 2 / nx:.6g}, {ly * (ja + jb) / 2 / ny:.6g}, {lz * (ka + kb) / 2 / nz:.6g}\n")
tet = kind in ("C3D4", "C3D10")
coords = {}
def xyz(p):
    i, j, k = p
    return (lx * i / nx, ly * j / ny, lz * k / nz)
if tet:
    mid = {}
    n = (nx + 1) * (ny + 1) * (nz + 1)
    extra = []
TET_SPLIT = [(0, 1, 2, 6), (0, 2, 3, 6), (0, 3, 7, 6), (0, 7, 4, 6), (0, 4, 5, 6), (0, 5, 1, 6)]
TET_EDGES = [(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)]
def oriented(t):
    import itertools
    a, b, c, d = (xyz(p) for p in t)
    u = [b[i] - a[i] for i in range(3)]
    v = [c[i] - a[i] for i in range(3)]
    w = [d[i] - a[i] for i in range(3)]
    cross = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
    if sum(cross[i] * w[i] for i in range(3)) < 0:
        return (t[0], t[2], t[1], t[3])
    return t
def tet_mid(a, b):
    key = (min(a, b), max(a, b))
    if key not in mid:
        global n
        n += 1
        mid[key] = n
        pa, pb = xyz(a), xyz(b)
        extra.append(f"{n}, {(pa[0] + pb[0]) / 2:.6g}, {(pa[1] + pb[1]) / 2:.6g}, {(pa[2] + pb[2]) / 2:.6g}\n")
    return mid[key]
elements = []
e = 0
for k in range(nz):
    for j in range(ny):
        for i in range(nx):
            c = corners(i, j, k)
            if not tet:
                e += 1
                ids = [nid(*p) for p in c]
                if kind == "C3D20R":
                    ids += [mid[(min(c[a], c[b]), max(c[a], c[b]))] for a, b in edges]
                elements.append((e, ids))
            else:
                for split in TET_SPLIT:
                    e += 1
                    t = oriented(tuple(c[q] for q in split))
                    ids = [nid(*p) for p in t]
                    if kind == "C3D10":
                        ids += [tet_mid(t[a], t[b]) for a, b in TET_EDGES]
                    elements.append((e, ids))
if tet:
    out.writelines(extra)
out.write(f"*ELEMENT, TYPE={kind}, ELSET=EALL\n")
for e, ids in elements:
    line = f"{e}, " + ", ".join(str(x) for x in ids[:15])
    if len(ids) > 15:
        line += ",\n" + ", ".join(str(x) for x in ids[15:])
    out.write(line + "\n")
out.write("*NSET, NSET=FIX\n")
for k in range(nz + 1):
    for j in range(ny + 1):
        out.write(f"{nid(0, j, k)}\n")
for (a, b), m in mid.items():
    if a[0] == 0 and b[0] == 0:
        out.write(f"{m}\n")
out.write("*NSET, NSET=TOP\n")
top = [nid(i, j, nz) for j in range(ny + 1) for i in range(nx + 1)]
if kind in ("C3D20R", "C3D10"):
    top += [m for (a, b), m in mid.items() if a[2] == nz and b[2] == nz]
for t in top:
    out.write(f"{t}\n")
out.write("*MATERIAL, NAME=STEEL\n*ELASTIC\n210000, 0.3\n*DENSITY\n7.85e-9\n")
out.write("*SOLID SECTION, ELSET=EALL, MATERIAL=STEEL\n")
out.write("*BOUNDARY\nFIX, 1, 3\n")
out.write("*STEP\n*STATIC" + (", SOLVER=ITERATIVE CHOLESKY" if solver == "iterative" else "") + "\n")
out.write(f"*CLOAD\nTOP, 3, {-1000.0 / len(top):.6g}\n")
out.write("*NODE FILE\nU\n*EL FILE\nS\n*END STEP\n")
