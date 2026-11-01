#!/usr/bin/env python3
"""Builds the gear the program icon is drawn from and solves it with CalculiX.

Writes, next to this script:
  part/gear.inp   hexahedral mesh (C3D8I) with a static torque step, a normal CalculiX input
  part/gear.val   one line "node value" per node, the result the icon is coloured with
                  (displacement magnitude, scaled to [0, 1])

nodalis-icon.typ reads both. Only needed when the part changes, the outputs are checked in.

    python3 assets/icon/make_part.py          # needs ccx on PATH
"""

import math
import pathlib
import subprocess
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
PART = HERE / "part"


class Mesh:
    def __init__(self):
        self.nodes = {}  # key -> (id, (x, y, z))
        self.elements = []

    def node(self, key, xyz):
        if key not in self.nodes:
            self.nodes[key] = (len(self.nodes) + 1, xyz)
        return self.nodes[key][0]

    def by_id(self):
        return {nid: xyz for nid, xyz in self.nodes.values()}


def polar(r, a, z):
    return (r * math.cos(a), r * math.sin(a), z)


def extrude(mesh, cells, layers, z0, z1, tag):
    """cells: list of ((key, xy) * 4) counter-clockwise quads; extruded into hexahedra."""
    for quad in cells:
        ids = []
        for k in range(layers + 1):
            z = z0 + (z1 - z0) * k / layers
            ids.append([mesh.node((tag, key, round(z, 9)), (xy[0], xy[1], z)) for key, xy in quad])
        for k in range(layers):
            mesh.elements.append(ids[k] + ids[k + 1])


# ------------------------------------------------------------------ gear


def gear():
    """Spur gear with involute teeth and root fillets, a thin web with spokes and windows
    between rim and hub, and a hub that stands out on both sides."""
    z_teeth, module, alpha = 16, 5.0, math.radians(20)
    r_pitch = module * z_teeth / 2
    r_tip, r_root, r_base = r_pitch + module, r_pitch - 1.25 * module, r_pitch * math.cos(alpha)
    r_bore, r_hub, r_rim = 8.0, 15.0, r_root - 5.5
    spokes = 5
    pitch = 2 * math.pi / z_teeth
    inv = lambda a: math.tan(a) - a

    def half_angle(r):
        """Half the angular tooth thickness at radius r: involute flank, root fillet below."""
        rr = max(r, r_base)
        psi = pitch / 4 + inv(alpha) - inv(math.acos(min(1.0, r_base / rr)))
        fillet = 0.35 * module
        if r < r_root + fillet:
            # Quarter circle that widens the tooth into the root.
            d = (r_root + fillet - r) / fillet
            psi += (1 - math.sqrt(max(0.0, 1 - d * d))) * fillet * 0.9 / r
        return psi

    n_tooth, across, gap_cells = 7, 4, 4
    psi0 = half_angle(r_root)

    # Angles of the nodes along the root circle within one pitch: across the tooth base,
    # then across the gap; inner rings reuse them so everything stays conforming.
    def ring_angle(t, j):
        mid = t * pitch
        if j <= across:
            return mid - psi0 + 2 * psi0 * j / across
        return mid + psi0 + (pitch - 2 * psi0) * (j - across) / gap_cells

    per_pitch = across + gap_cells
    n_ring = z_teeth * per_pitch

    def angle(j):
        t, k = divmod(j % n_ring, per_pitch)
        return ring_angle(t, k)

    radii = [r_bore + (r_hub - r_bore) * i / 3 for i in range(3)]
    radii += [r_hub + (r_rim - r_hub) * i / 5 for i in range(5)]
    radii += [r_rim + (r_root - r_rim) * i / 3 for i in range(4)]
    zone = lambda i: "hub" if radii[i] < r_hub - 1e-9 else ("web" if radii[i] < r_rim - 1e-9 else "rim")

    def ring(i, j):
        return (("r", i, j % n_ring), polar(radii[i], angle(j), 0)[:2])

    def tooth(t, i, j):
        if i == 0:
            return ring(len(radii) - 1, t * per_pitch + j)
        u = i / n_tooth
        r = r_root + (r_tip - r_root) * (u ** 0.85)
        psi = half_angle(r)
        a = t * pitch - psi + 2 * psi * j / across
        return (("t", t, i, j), polar(r, a, 0)[:2])

    cells = {"hub": [], "web": [], "rim": []}
    for i in range(len(radii) - 1):
        for j in range(n_ring):
            a_mid = (angle(j) + angle(j + 1)) / 2 if j + 1 < n_ring else angle(j) + 0.01
            kind = zone(i)
            if kind == "web":
                # Spokes stay solid, the windows between them are left open.
                sector = (a_mid % (2 * math.pi)) / (2 * math.pi / spokes)
                if abs(sector - round(sector)) > 0.16 and 0 < i - 3 < 4:
                    continue
            cells[kind].append([ring(i, j), ring(i + 1, j), ring(i + 1, j + 1), ring(i, j + 1)])
    for t in range(z_teeth):
        for i in range(n_tooth):
            for j in range(across):
                cells["rim"].append([tooth(t, i, j), tooth(t, i + 1, j), tooth(t, i + 1, j + 1), tooth(t, i, j + 1)])

    mesh = Mesh()
    width, web, hub = 14.0, 4.0, 3.0  # face width, web thickness, hub overhang per side
    def stack(quads, zs):
        for k in range(len(zs) - 1):
            extrude(mesh, quads, 1, zs[k], zs[k + 1], "g")
    mid0, mid1 = (width - web) / 2, (width + web) / 2
    # Layers line up with the web faces so rim, web and hub share their nodes.
    rim_z = [0.0, mid0, mid1, width]
    stack(cells["rim"], rim_z)
    stack(cells["web"], [mid0, mid1])
    stack(cells["hub"], [-hub] + rim_z + [width + hub])

    nodes = mesh.by_id()
    fixed = [n for n, (x, y, z) in nodes.items() if math.hypot(x, y) < r_bore + 1e-6]
    # Torque: a tangential force on one flank of every tooth (the j = across side).
    flank = [mesh.nodes[k][0] for k in mesh.nodes if k[1][0] == "t" and k[1][3] == across]
    loads = []
    for n in flank:
        x, y, _ = nodes[n]
        a = math.atan2(y, x)
        f = -6000.0 / len(flank)
        loads += [(n, 1, -f * math.sin(a)), (n, 2, f * math.cos(a))]
    return mesh, fixed, loads, None, "displacement"


# ------------------------------------------------------------------ CalculiX


def write_inp(path, mesh, fixed, loads, omega):
    nodes = mesh.by_id()
    lines = ["** Nodalis icon part, written by assets/icon/make_parts.py", "*NODE, NSET=NALL"]
    lines += [f"{n}, {x:.6f}, {y:.6f}, {z:.6f}" for n, (x, y, z) in sorted(nodes.items())]
    lines.append("*ELEMENT, TYPE=C3D8I, ELSET=EALL")
    lines += [f"{e}, " + ", ".join(map(str, ids)) for e, ids in enumerate(mesh.elements, 1)]
    lines.append("*NSET, NSET=FIXED")
    lines += [f"{n}," for n in fixed]
    lines += [
        "*MATERIAL, NAME=STEEL",
        "*ELASTIC",
        "210000, 0.3",
        "*DENSITY",
        "7.85e-9",
        "*SOLID SECTION, ELSET=EALL, MATERIAL=STEEL",
        "*STEP",
        "*STATIC",
        "*BOUNDARY",
        "FIXED, 1, 3",
    ]
    if loads:
        lines.append("*CLOAD")
        lines += [f"{n}, {d}, {f:.6f}" for n, d, f in loads]
    if omega:
        lines += ["*DLOAD", f"EALL, CENTRIF, {omega**2:.1f}, 0, 0, 0, 0, 0, 1"]
    lines += ["*NODE FILE", "U", "*EL FILE", "S", "*END STEP", ""]
    path.write_text("\n".join(lines))


def read_frd(path):
    """Nodal displacement magnitude and von Mises stress from a CalculiX .frd file."""
    disp, mises, block = {}, {}, None
    for line in path.read_text().splitlines():
        if line.startswith(" -4"):
            block = line.split()[1]
        elif line.startswith(" -1") and block in ("DISP", "STRESS"):
            n = int(line[3:13])
            v = [float(line[13 + 12 * k : 25 + 12 * k]) for k in range((len(line) - 13) // 12)]
            if block == "DISP":
                disp[n] = math.sqrt(sum(c * c for c in v[:3]))
            else:
                sx, sy, sz, sxy, syz, szx = v[:6]
                mises[n] = math.sqrt(
                    0.5 * ((sx - sy) ** 2 + (sy - sz) ** 2 + (sz - sx) ** 2)
                    + 3 * (sxy**2 + syz**2 + szx**2)
                )
        elif line.startswith(" -3"):
            block = None
    return {"displacement": disp, "mises": mises}


def main():
    PART.mkdir(exist_ok=True)
    mesh, fixed, loads, omega, field = gear()
    inp = PART / "gear.inp"
    write_inp(inp, mesh, fixed, loads, omega)
    with tempfile.TemporaryDirectory() as tmp:
        job = pathlib.Path(tmp) / "gear"
        job.with_suffix(".inp").write_text(inp.read_text())
        subprocess.run(["ccx", "-i", str(job)], cwd=tmp, check=True, capture_output=True)
        values = read_frd(job.with_suffix(".frd"))[field]
    top = max(values.values())
    (PART / "gear.val").write_text("".join(f"{n} {v / top:.4f}\n" for n, v in sorted(values.items())))
    print(f"gear: {len(mesh.nodes)} nodes, {len(mesh.elements)} elements, max {field} {top:.4g}")


if __name__ == "__main__":
    main()
