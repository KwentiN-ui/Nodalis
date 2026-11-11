#!/usr/bin/env python3
"""Dynamic-step benchmark inputs for plxs, shaped like the Nodalis lowering of a Dynamic step.

    gen_dynamic.py <case> > job.inp

Cases (mm, N, t, s; steel):
  beam-<kind>-<nx>x<ny>x<nz>-f<frames>-p<per_frame>[-damp][-nlgeom]
      cantilever block 100 x 20 x 20 mm, clamped at x = 0, step load (pressure) on the top
      face, `frames` result frames over 0.02 s, `per_frame` time steps per frame
      (-damp: *DAMPING, RATIO; -nlgeom: NLGEOM with a load 100x larger)
  drop-<n>-f<frames>
      block of n x n x n C3D8 (10 mm cubes) dropped on a fixed plate (20x20x2 C3D8), node to
      surface contact, gravity + initial velocity
"""
import sys


def block_mesh(nx, ny, nz, lx, ly, lz, kind, first_node=1, first_el=1, x0=0.0, y0=0.0, z0=0.0):
    """Returns (nodes: list[(id,x,y,z)], elements: list[(id, ids)], sets dict)."""
    nodes = []
    def nid(i, j, k):
        return first_node + i + (nx + 1) * (j + (ny + 1) * k)
    for k in range(nz + 1):
        for j in range(ny + 1):
            for i in range(nx + 1):
                nodes.append((nid(i, j, k), x0 + lx * i / nx, y0 + ly * j / ny, z0 + lz * k / nz))
    n = first_node + (nx + 1) * (ny + 1) * (nz + 1) - 1
    mid = {}

    def xyz(p):
        i, j, k = p
        return (x0 + lx * i / nx, y0 + ly * j / ny, z0 + lz * k / nz)

    def corners(i, j, k):
        return [(i, j, k), (i + 1, j, k), (i + 1, j + 1, k), (i, j + 1, k),
                (i, j, k + 1), (i + 1, j, k + 1), (i + 1, j + 1, k + 1), (i, j + 1, k + 1)]

    def midnode(a, b):
        nonlocal n
        key = (min(a, b), max(a, b))
        if key not in mid:
            n += 1
            mid[key] = n
            pa, pb = xyz(a), xyz(b)
            nodes.append((n, (pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2, (pa[2] + pb[2]) / 2))
        return mid[key]

    TET_SPLIT = [(0, 1, 2, 6), (0, 2, 3, 6), (0, 3, 7, 6), (0, 7, 4, 6), (0, 4, 5, 6), (0, 5, 1, 6)]
    TET_EDGES = [(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)]
    HEX_EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
                 (0, 4), (1, 5), (2, 6), (3, 7)]

    def oriented(t):
        a, b, c, d = (xyz(p) for p in t)
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        w = [d[i] - a[i] for i in range(3)]
        cross = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
        if sum(cross[i] * w[i] for i in range(3)) < 0:
            return (t[0], t[2], t[1], t[3])
        return t

    elements = []
    e = first_el - 1
    tet = kind in ("C3D4", "C3D10")
    for k in range(nz):
        for j in range(ny):
            for i in range(nx):
                c = corners(i, j, k)
                if not tet:
                    e += 1
                    ids = [nid(*p) for p in c]
                    if kind == "C3D20R" or kind == "C3D20":
                        ids += [midnode(c[a], c[b]) for a, b in HEX_EDGES]
                    elements.append((e, ids))
                else:
                    for split in TET_SPLIT:
                        e += 1
                        t = oriented(tuple(c[q] for q in split))
                        ids = [nid(*p) for p in t]
                        if kind == "C3D10":
                            ids += [midnode(t[a], t[b]) for a, b in TET_EDGES]
                        elements.append((e, ids))
    sets = {}
    sets["x0"] = [nid(0, j, k) for k in range(nz + 1) for j in range(ny + 1)] + \
        [m for (a, b), m in mid.items() if a[0] == 0 and b[0] == 0]
    sets["top"] = [nid(i, j, nz) for j in range(ny + 1) for i in range(nx + 1)] + \
        [m for (a, b), m in mid.items() if a[2] == nz and b[2] == nz]
    sets["bottom"] = [nid(i, j, 0) for j in range(ny + 1) for i in range(nx + 1)] + \
        [m for (a, b), m in mid.items() if a[2] == 0 and b[2] == 0]
    sets["all"] = [nd[0] for nd in nodes]
    # Elements touching the top / bottom face (hex: one layer; tets: those with 3 nodes there).
    top_ids = set(nid(i, j, nz) for j in range(ny + 1) for i in range(nx + 1))
    bot_ids = set(nid(i, j, 0) for j in range(ny + 1) for i in range(nx + 1))
    sets["top_el"] = [eid for eid, ids in elements if len(top_ids & set(ids[:8])) >= (4 if not tet else 3)]
    sets["bottom_el"] = [eid for eid, ids in elements if len(bot_ids & set(ids[:8])) >= (4 if not tet else 3)]
    return nodes, elements, sets


def write_mesh(out, nodes, elements, kind, elset):
    out.write("*NODE\n")
    for nd in nodes:
        out.write(f"{nd[0]}, {nd[1]:.6g}, {nd[2]:.6g}, {nd[3]:.6g}\n")
    out.write(f"*ELEMENT, TYPE={kind}, ELSET={elset}\n")
    for e, ids in elements:
        line = f"{e}, " + ", ".join(str(x) for x in ids[:15])
        if len(ids) > 15:
            line += ",\n" + ", ".join(str(x) for x in ids[15:])
        out.write(line + "\n")


def write_set(out, card, name, ids):
    out.write(f"*{card}, {card}={name}\n")
    for i in range(0, len(ids), 12):
        out.write(", ".join(str(x) for x in ids[i:i + 12]) + "\n")


def beam(out, kind, nx, ny, nz, frames, per_frame, damp, nlgeom, duration=0.02):
    nodes, elements, sets = block_mesh(nx, ny, nz, 100.0, 20.0, 20.0, kind)
    write_mesh(out, nodes, elements, kind, "EALL")
    write_set(out, "NSET", "FIX", sets["x0"])
    write_set(out, "ELSET", "TOPEL", sets["top_el"])
    out.write("*SURFACE, NAME=TOP, TYPE=ELEMENT\n")
    # Face of the top: for hexes S6 (k+1 face: nodes 5-8); for the tets built here the face
    # at z = lz varies, so load the top nodes with CLOAD instead.
    out.write("*MATERIAL, NAME=STEEL\n*ELASTIC\n210000, 0.3\n*DENSITY\n7.85e-9\n")
    out.write("*SOLID SECTION, ELSET=EALL, MATERIAL=STEEL\n")
    if damp:
        pass
    dt = duration / frames / per_frame
    out.write(f"*STEP, INC={frames * per_frame + 1}" + (", NLGEOM" if nlgeom else "") + "\n")
    out.write(f"*DYNAMIC, DIRECT, ALPHA=-0.05\n{dt:.10g}, {duration}\n")
    if damp:
        out.write(f"*DAMPING, RATIO=0.02, UPPER={1.0 / (10.0 * dt):.6g}\n")
    out.write("*BOUNDARY\nFIX, 1, 3\n")
    total = -200.0 * (100.0 if nlgeom else 1.0)  # N on the top face, step load
    write_set(out, "NSET", "TOPN", sets["top"])
    out.write(f"*CLOAD\nTOPN, 3, {total / len(sets['top']):.6g}\n")
    out.write(f"*NODE FILE, FREQUENCY={per_frame}\nU, V, A, RF\n")
    out.write(f"*EL FILE, FREQUENCY={per_frame}\nS, E\n")
    out.write("*EL PRINT, ELSET=EALL, TOTALS=ONLY\nALLIE, ALLKE, ALLWK, ALLVD\n")
    out.write("*END STEP\n")


def drop(out, n, frames, duration=0.002):
    # Plate 200 x 200 x 20 mm, 20 x 20 x 2 C3D8, bottom fixed.
    pn, pe, ps = block_mesh(20, 20, 2, 200.0, 200.0, 20.0, "C3D8")
    # Block n x n x n of 10 mm cubes centred above the plate, gap 0.05 mm.
    side = 10.0 * n
    bn, be, bs = block_mesh(n, n, n, side, side, side, "C3D8", first_node=100001, first_el=100001,
                            x0=100.0 - side / 2, y0=100.0 - side / 2, z0=20.0 + 0.05)
    out.write("*NODE\n")
    for nd in pn + bn:
        out.write(f"{nd[0]}, {nd[1]:.6g}, {nd[2]:.6g}, {nd[3]:.6g}\n")
    out.write("*ELEMENT, TYPE=C3D8, ELSET=PLATE\n")
    for e, ids in pe:
        out.write(f"{e}, " + ", ".join(str(x) for x in ids) + "\n")
    out.write("*ELEMENT, TYPE=C3D8, ELSET=BLOCK\n")
    for e, ids in be:
        out.write(f"{e}, " + ", ".join(str(x) for x in ids) + "\n")
    out.write("*ELSET, ELSET=EALL\nPLATE, BLOCK\n")
    write_set(out, "NSET", "FIX", ps["bottom"])
    write_set(out, "NSET", "BLOCKN", bs["all"])
    write_set(out, "ELSET", "PLATETOP", ps["top_el"])
    write_set(out, "ELSET", "BLOCKBOT", bs["bottom_el"])
    # ccx hex faces: S1 bottom (1-2-3-4), S2 top (5-6-7-8).
    out.write("*SURFACE, NAME=MASTER, TYPE=ELEMENT\nPLATETOP, S2\n")
    out.write("*SURFACE, NAME=SLAVE, TYPE=ELEMENT\nBLOCKBOT, S1\n")
    out.write("*MATERIAL, NAME=STEEL\n*ELASTIC\n210000, 0.3\n*DENSITY\n7.85e-9\n")
    out.write("*SOLID SECTION, ELSET=EALL, MATERIAL=STEEL\n")
    out.write("*SURFACE INTERACTION, NAME=SI\n*SURFACE BEHAVIOR, PRESSURE-OVERCLOSURE=LINEAR\n1e6\n")
    out.write("*CONTACT PAIR, INTERACTION=SI, TYPE=NODE TO SURFACE\nSLAVE, MASTER\n")
    out.write("*INITIAL CONDITIONS, TYPE=VELOCITY\nBLOCKN, 3, -1000\n")
    dt = duration / frames
    out.write(f"*STEP, INC={frames + 1}\n*DYNAMIC, DIRECT\n{dt:.10g}, {duration}\n")
    out.write("*BOUNDARY\nFIX, 1, 3\n")
    out.write("*DLOAD\nEALL, GRAV, 9810, 0, 0, -1\n")
    out.write("*NODE FILE\nU, V, A, RF\n*EL FILE\nS, E\n")
    out.write("*CONTACT FILE\nCDIS, CSTR\n")
    out.write("*EL PRINT, ELSET=EALL, TOTALS=ONLY\nALLIE, ALLKE, ALLWK, ALLVD\n")
    out.write("*END STEP\n")


def main():
    case = sys.argv[1]
    parts = case.split("-")
    out = sys.stdout
    if parts[0] == "beam":
        kind = parts[1]
        nx, ny, nz = (int(v) for v in parts[2].split("x"))
        frames = int(parts[3][1:])
        per_frame = int(parts[4][1:])
        beam(out, kind, nx, ny, nz, frames, per_frame, "damp" in parts, "nlgeom" in parts)
    elif parts[0] == "drop":
        drop(out, int(parts[1]), int(parts[2][1:]))
    else:
        sys.exit("unknown case")


main()
