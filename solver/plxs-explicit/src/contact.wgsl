// Penalty contact on the card: one thread per slave point (a slave node, node-to-face, or
// an integration point of a slave face, face-to-face), the same steps as
// `plxs_contact::Pair::evaluate_point` with the damping and the stiffness cap of the
// explicit integration: the closest of the point's candidate master faces, the gap, the
// pressure-overclosure relation, Coulomb friction with return mapping, the damping of a
// pressed spring. The forces go into slots, one per node a point can act on (its own
// nodes and those of its candidate faces); `contact_rows` sums the Gershgorin rows of the
// springs per DOF, `contact_cap` lowers the springs too stiff for the time step,
// `contact_gather` adds the forces per DOF. All in double precision but `atan` and `exp`
// (single precision, relative error 1e-7), and square roots by Newton from single
// precision (DirectX has none in double precision).
//
// `contact_check` flags a search for candidates (a contact node moved farther than the
// search covers) and `contact_gate`, first in every time step, stops the run after the
// time step that flagged it or whose contact opened or closed (for an event frame): it
// zeroes the indirect dispatches of all later passes.

struct CDims {
    points: u32,
    n: u32,
    events_from: u32,
    arg_words: u32,
    face_base: u32,
    point_base: u32,
    table_base: u32,
    width_base: u32,
    pdata_base: u32,
    cand_base: u32,
    slot_ptr_base: u32,
    slot_base: u32,
    gnodes: u32,
    gnode_base: u32,
    nslot_ptr_base: u32,
    nslot_base: u32,
    checks: u32,
    check_base: u32,
    pad0: u32,
    pad1: u32,
}

struct Step {
    h: f64,
    c: f64,
    s: f64,
    alpha: f64,
    s_before: f64,
    // The index of the time step in the run.
    index: f64,
}

@group(0) @binding(0) var<uniform> cd: CDims;
@group(0) @binding(1) var<uniform> step: Step;
// Faces (stride 10: 8 nodes, node count, kind) and points (stride 10: 8 nodes, node
// count, pair).
@group(0) @binding(2) var<storage, read> ints: array<u32>;
// Constants (0 reach, 1 stiffness cap, 2 damping, 3 rounding, 4 beside tolerance), pairs
// (stride 16 from 8), tables, face widths, point data (stride 11: 8 weights, area,
// cutoff, initial gap).
@group(0) @binding(3) var<storage, read> reals: array<f64>;
// Candidates (pointers, faces), slots (pointers, nodes), gathered nodes (nodes, pointers
// to their slots, slots), checked nodes.
@group(0) @binding(4) var<storage, read> lists: array<u32>;
// The positions of the checked nodes at the last search.
@group(0) @binding(5) var<storage, read> prepared: array<f64>;
@group(0) @binding(6) var<storage, read> coords: array<f64>;
@group(0) @binding(7) var<storage, read> u: array<f64>;
@group(0) @binding(8) var<storage, read> v: array<f64>;
// Per point (stride 8): status (0 open, 1 stick, 2 slip), slip 1, slip 2, stiffness scale,
// gap, pressure, master face (-1 without a spring).
@group(0) @binding(9) var<storage, read_write> state: array<f64>;
// Per slot (stride 9): spring force, its damping part, Gershgorin row.
@group(0) @binding(10) var<storage, read_write> slots: array<f64>;
// The Gershgorin rows of the springs per DOF; then `1/√m` per DOF (0 for a DOF that does
// not move).
@group(0) @binding(11) var<storage, read_write> rows: array<f64>;
// The internal and the viscous forces per DOF.
@group(0) @binding(12) var<storage, read_write> fs: array<f64>;
// 0 a contact opened or closed, 1 a search is due, 2 the time step after which the run
// stopped (+1), 3 why (1 search, 2 event), 4 a contact opened or closed in the last time
// step.
@group(0) @binding(13) var<storage, read_write> flags: array<atomic<u32>>;
@group(0) @binding(14) var<storage, read_write> args: array<u32>;

const PAIR: u32 = 16u;
const PAIRS: u32 = 8u;

var<private> sn: array<f64, 8>;
var<private> sdn: array<vec2<f64>, 8>;

// `abs`, `max`, `min` and `clamp` in double precision by comparison: the builtins crashed the
// software renderer (llvmpipe) in loops.
fn max64(a: f64, b: f64) -> f64 {
    return select(b, a, a > b);
}
fn min64(a: f64, b: f64) -> f64 {
    return select(b, a, a < b);
}
fn clamp64(x: f64, lo: f64, hi: f64) -> f64 {
    return min64(max64(x, lo), hi);
}
fn abs64(x: f64) -> f64 {
    return select(-x, x, x >= f64(0.0));
}
fn max3(a: vec3<f64>, b: vec3<f64>) -> vec3<f64> {
    return vec3<f64>(max64(a.x, b.x), max64(a.y, b.y), max64(a.z, b.z));
}
fn min3(a: vec3<f64>, b: vec3<f64>) -> vec3<f64> {
    return vec3<f64>(min64(a.x, b.x), min64(a.y, b.y), min64(a.z, b.z));
}

fn sqrt64(x: f64) -> f64 {
    if x <= f64(0.0) {
        return f64(0.0);
    }
    var y = f64(sqrt(f32(x)));
    if y == f64(0.0) {
        // Below the range of single precision.
        y = f64(1e-30);
    }
    for (var k = 0; k < 3; k++) {
        y = f64(0.5) * (y + x / y);
    }
    return y;
}

fn dot3(a: vec3<f64>, b: vec3<f64>) -> f64 {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

fn cross3(a: vec3<f64>, b: vec3<f64>) -> vec3<f64> {
    return vec3<f64>(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

fn norm3(a: vec3<f64>) -> f64 {
    return sqrt64(dot3(a, a));
}

fn normalized3(a: vec3<f64>) -> vec3<f64> {
    let l = norm3(a);
    if l > f64(0.0) {
        return a * (f64(1.0) / l);
    }
    return a;
}

fn current(node: u32) -> vec3<f64> {
    let k = 3u * node;
    return vec3<f64>(coords[k] + u[k], coords[k + 1u] + u[k + 1u], coords[k + 2u] + u[k + 2u]);
}

fn pr(pair: u32, k: u32) -> f64 {
    return reals[PAIRS + pair * PAIR + k];
}

// Face data.
fn face_count(f: u32) -> u32 {
    return ints[cd.face_base + f * 10u + 8u];
}
fn face_node(f: u32, i: u32) -> u32 {
    return ints[cd.face_base + f * 10u + i];
}
fn face_corners(f: u32) -> u32 {
    return ints[cd.face_base + f * 10u + 9u] & 15u;
}
fn face_quadratic(f: u32) -> bool {
    return ((ints[cd.face_base + f * 10u + 9u] >> 4u) & 1u) == 1u;
}
// 0 a face, 1 an edge of a plane element, 2 an edge of an axisymmetric one.
fn face_planar(f: u32) -> u32 {
    return (ints[cd.face_base + f * 10u + 9u] >> 8u) & 3u;
}

// Shape functions and their natural derivatives (`ContactFace::shape`) into `sn`, `sdn`.
fn shape(f: u32, p: vec2<f64>) {
    let corners = face_corners(f);
    let quadratic = face_quadratic(f);
    let x = p.x;
    let y = p.y;
    let one = f64(1.0);
    let half = f64(0.5);
    let quarter = f64(0.25);
    if corners == 2u {
        if quadratic {
            sn[0] = half * x * (x - one);
            sn[1] = half * x * (x + one);
            sn[2] = one - x * x;
            sdn[0] = vec2<f64>(x - half, f64(0.0));
            sdn[1] = vec2<f64>(x + half, f64(0.0));
            sdn[2] = vec2<f64>(f64(-2.0) * x, f64(0.0));
        } else {
            sn[0] = half * (one - x);
            sn[1] = half * (one + x);
            sdn[0] = vec2<f64>(-half, f64(0.0));
            sdn[1] = vec2<f64>(half, f64(0.0));
        }
        return;
    }
    if corners == 4u {
        var cx = array<f64, 4>(-one, one, one, -one);
        var cy = array<f64, 4>(-one, -one, one, one);
        if !quadratic {
            for (var i = 0u; i < 4u; i++) {
                sn[i] = quarter * (one + cx[i] * x) * (one + cy[i] * y);
                sdn[i] = vec2<f64>(quarter * cx[i] * (one + cy[i] * y), quarter * cy[i] * (one + cx[i] * x));
            }
        } else {
            for (var i = 0u; i < 4u; i++) {
                let a = one + cx[i] * x;
                let b = one + cy[i] * y;
                let s = cx[i] * x + cy[i] * y - one;
                sn[i] = quarter * a * b * s;
                sdn[i] = vec2<f64>(quarter * cx[i] * b * (s + a), quarter * cy[i] * a * (s + b));
            }
            var mx = array<f64, 4>(f64(0.0), one, f64(0.0), -one);
            var my = array<f64, 4>(-one, f64(0.0), one, f64(0.0));
            for (var m = 0u; m < 4u; m++) {
                let i = 4u + m;
                if mx[m] == f64(0.0) {
                    sn[i] = half * (one - x * x) * (one + my[m] * y);
                    sdn[i] = vec2<f64>(-x * (one + my[m] * y), half * (one - x * x) * my[m]);
                } else {
                    sn[i] = half * (one + mx[m] * x) * (one - y * y);
                    sdn[i] = vec2<f64>(half * mx[m] * (one - y * y), -y * (one + mx[m] * x));
                }
            }
        }
        return;
    }
    var l = array<f64, 3>(one - x - y, x, y);
    var dl = array<vec2<f64>, 3>(vec2<f64>(-one, -one), vec2<f64>(one, f64(0.0)), vec2<f64>(f64(0.0), one));
    if !quadratic {
        for (var i = 0u; i < 3u; i++) {
            sn[i] = l[i];
            sdn[i] = dl[i];
        }
    } else {
        for (var i = 0u; i < 3u; i++) {
            sn[i] = l[i] * (f64(2.0) * l[i] - one);
            sdn[i] = (f64(4.0) * l[i] - one) * dl[i];
        }
        var ea = array<u32, 3>(0u, 1u, 2u);
        var eb = array<u32, 3>(1u, 2u, 0u);
        for (var m = 0u; m < 3u; m++) {
            let a = ea[m];
            let b = eb[m];
            sn[3u + m] = f64(4.0) * l[a] * l[b];
            sdn[3u + m] = f64(4.0) * (dl[a] * l[b] + l[a] * dl[b]);
        }
    }
}

struct Place {
    x: vec3<f64>,
    t0: vec3<f64>,
    t1: vec3<f64>,
}

// The point and the tangents at natural coordinates (`ContactFace::place`).
fn place(f: u32, p: vec2<f64>) -> Place {
    shape(f, p);
    var x = vec3<f64>(f64(0.0), f64(0.0), f64(0.0));
    var t0 = x;
    var t1 = x;
    let m = face_count(f);
    for (var i = 0u; i < m; i++) {
        let c = current(face_node(f, i));
        x += sn[i] * c;
        t0 += sdn[i].x * c;
        t1 += sdn[i].y * c;
    }
    if face_planar(f) != 0u {
        t1 = vec3<f64>(f64(0.0), f64(0.0), f64(1.0));
    }
    return Place(x, t0, t1);
}

fn clamp_natural(f: u32, p: vec2<f64>) -> vec2<f64> {
    let corners = face_corners(f);
    let one = f64(1.0);
    if corners == 2u {
        return vec2<f64>(clamp64(p.x, -one, one), f64(0.0));
    }
    if corners == 4u {
        return vec2<f64>(clamp64(p.x, -one, one), clamp64(p.y, -one, one));
    }
    var x = max64(p.x, f64(0.0));
    var y = max64(p.y, f64(0.0));
    let s = x + y;
    if s > one {
        x = x / s;
        y = y / s;
    }
    return vec2<f64>(x, y);
}

fn is_interior(f: u32, p: vec2<f64>) -> bool {
    let corners = face_corners(f);
    let e = f64(1e-9);
    let one = f64(1.0);
    if corners == 2u {
        return abs64(p.x) < one - e;
    }
    if corners == 4u {
        return abs64(p.x) < one - e && abs64(p.y) < one - e;
    }
    return p.x > e && p.y > e && p.x + p.y < one - e;
}

// The closest point projection of `x` on the face, clamped (`project_natural`).
fn project_natural(f: u32, x: vec3<f64>) -> vec2<f64> {
    var p = vec2<f64>(f64(0.0), f64(0.0));
    if face_corners(f) == 3u {
        p = vec2<f64>(f64(1.0) / f64(3.0), f64(1.0) / f64(3.0));
    }
    for (var it = 0; it < 30; it++) {
        let at = place(f, p);
        let r = x - at.x;
        let g0 = dot3(at.t0, r);
        let g1 = dot3(at.t1, r);
        let a = dot3(at.t0, at.t0);
        let b = dot3(at.t0, at.t1);
        let c = dot3(at.t1, at.t1);
        let det = a * c - b * b;
        if abs64(det) < f64(1e-300) {
            break;
        }
        let stp = vec2<f64>((c * g0 - b * g1) / det, (a * g1 - b * g0) / det);
        let next = clamp_natural(f, p + stp);
        let moved = abs64(next.x - p.x) + abs64(next.y - p.y);
        p = next;
        if moved < f64(1e-13) {
            break;
        }
    }
    return p;
}

// The distance of `x` from the bounding box of the face nodes.
fn box_distance(f: u32, x: vec3<f64>) -> f64 {
    let m = face_count(f);
    var lo = current(face_node(f, 0u));
    var hi = lo;
    for (var i = 1u; i < m; i++) {
        let c = current(face_node(f, i));
        lo = min3(lo, c);
        hi = max3(hi, c);
    }
    let zero = vec3<f64>(f64(0.0), f64(0.0), f64(0.0));
    let out = max3(max3(lo - x, x - hi), zero);
    return sqrt64(dot3(out, out));
}

fn point_count(p: u32) -> u32 {
    return ints[cd.point_base + p * 10u + 8u];
}
fn point_node(p: u32, i: u32) -> u32 {
    return ints[cd.point_base + p * 10u + i];
}
fn point_pair(p: u32) -> u32 {
    return ints[cd.point_base + p * 10u + 9u];
}
fn pdata(p: u32, k: u32) -> f64 {
    return reals[cd.pdata_base + p * 11u + k];
}

// Whether the face has a node of the point.
fn own(p: u32, f: u32) -> bool {
    let m = face_count(f);
    let k = point_count(p);
    for (var i = 0u; i < m; i++) {
        let node = face_node(f, i);
        for (var j = 0u; j < k; j++) {
            if point_node(p, j) == node {
                return true;
            }
        }
    }
    return false;
}

fn candidates_from(p: u32) -> u32 {
    return lists[cd.cand_base + p];
}
fn candidates_to(p: u32) -> u32 {
    return lists[cd.cand_base + p + 1u];
}
fn candidate(k: u32) -> u32 {
    return lists[cd.cand_base + cd.points + 1u + k];
}

// Whether the edge or corner of face `f` that a clamped foot point lies on is shared with
// another candidate face (`Pair::inner_edge`): `sn` holds the shape functions there.
fn inner_edge(p: u32, f: u32) -> bool {
    let m = face_count(f);
    var edge = array<u32, 8>();
    var count = 0u;
    for (var i = 0u; i < m; i++) {
        if abs64(sn[i]) > f64(1e-9) {
            edge[count] = face_node(f, i);
            count++;
        }
    }
    if count == 0u {
        return false;
    }
    for (var k = candidates_from(p); k < candidates_to(p); k++) {
        let g = candidate(k);
        if g == f {
            continue;
        }
        let mg = face_count(g);
        var all = true;
        for (var e = 0u; e < count; e++) {
            var found = false;
            for (var j = 0u; j < mg; j++) {
                if face_node(g, j) == edge[e] {
                    found = true;
                }
            }
            if !found {
                all = false;
                break;
            }
        }
        if all {
            return true;
        }
    }
    return false;
}

fn tabular(pair: u32, d: f64) -> vec2<f64> {
    let start = u32(f32(pr(pair, 4u)));
    let len = u32(f32(pr(pair, 5u)));
    let zero = f64(0.0);
    if len == 0u {
        return vec2<f64>(zero, zero);
    }
    let base = cd.table_base + start;
    let x0 = reals[base];
    let p0 = reals[base + 1u];
    if len == 1u {
        if d < x0 {
            return vec2<f64>(zero, zero);
        }
        return vec2<f64>(p0, zero);
    }
    if d < x0 {
        let x1 = reals[base + 2u];
        var s = zero;
        if x1 > x0 {
            s = (reals[base + 3u] - p0) / (x1 - x0);
        }
        return vec2<f64>(zero, s);
    }
    for (var i = 0u; i + 1u < len; i++) {
        let xa = reals[base + 2u * i];
        let pa = reals[base + 2u * i + 1u];
        let xb = reals[base + 2u * i + 2u];
        let pb = reals[base + 2u * i + 3u];
        if d <= xb {
            var s = zero;
            if xb > xa {
                s = (pb - pa) / (xb - xa);
            }
            return vec2<f64>(pa + s * (d - xa), s);
        }
    }
    let xa = reals[base + 2u * (len - 2u)];
    let pa = reals[base + 2u * (len - 2u) + 1u];
    let xb = reals[base + 2u * (len - 1u)];
    let pb = reals[base + 2u * (len - 1u) + 1u];
    var s = zero;
    if xb > xa {
        s = (pb - pa) / (xb - xa);
    }
    return vec2<f64>(pb, s);
}

fn exponential(pair: u32, d: f64) -> vec2<f64> {
    let c0 = pr(pair, 2u);
    let p0 = pr(pair, 3u);
    let b = f64(2.0) * f64(2.302585092994046) / c0;
    let p = p0 * f64(exp(f32(b * d)));
    return vec2<f64>(p, b * p);
}

// `Law::node_pressure`.
fn node_pressure(pair: u32, d: f64) -> vec2<f64> {
    let law = u32(f32(pr(pair, 1u)));
    if law == 1u {
        return exponential(pair, d);
    }
    if law == 2u {
        return tabular(pair, d);
    }
    let k = pr(pair, 2u);
    let sigma = pr(pair, 3u);
    let pi = f64(3.141592653589793);
    let kd = k * d;
    let x = kd / (pi * sigma);
    let at = f64(atan(f32(x)));
    return vec2<f64>(f64(0.5) * kd + kd / pi * at, f64(0.5) * k + k / pi * (at + x / (f64(1.0) + x * x)));
}

// `Law::face_pressure` (not tied).
fn face_pressure(pair: u32, d: f64) -> vec2<f64> {
    let zero = f64(0.0);
    if d <= zero {
        return vec2<f64>(zero, zero);
    }
    let law = u32(f32(pr(pair, 1u)));
    if law == 1u {
        return exponential(pair, d);
    }
    if law == 2u {
        return tabular(pair, d);
    }
    let k = pr(pair, 2u);
    return vec2<f64>(k * d, k);
}

fn slot_of(p: u32, node: u32) -> u32 {
    let first_slot = lists[cd.slot_ptr_base + p];
    let last_slot = lists[cd.slot_ptr_base + p + 1u];
    for (var s = first_slot; s < last_slot; s++) {
        if lists[cd.slot_base + s] == node {
            return s;
        }
    }
    return 0xffffffffu;
}

fn write_open(p: u32, gap: f64) {
    let zero = f64(0.0);
    state[8u * p] = zero;
    state[8u * p + 1u] = zero;
    state[8u * p + 2u] = zero;
    state[8u * p + 4u] = gap;
    state[8u * p + 5u] = zero;
    state[8u * p + 6u] = f64(-1.0);
}

@compute @workgroup_size(64)
fn contact_point(
    @builtin(workgroup_id) wid: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let p = (wid.y * groups.x + wid.x) * 64u + li;
    if p >= cd.points {
        return;
    }
    let zero = f64(0.0);
    let one = f64(1.0);
    // The slots start empty.
    let slot_from = lists[cd.slot_ptr_base + p];
    let slot_to = lists[cd.slot_ptr_base + p + 1u];
    for (var s = slot_from; s < slot_to; s++) {
        for (var k = 0u; k < 9u; k++) {
            slots[9u * s + k] = zero;
        }
    }
    let was_open = state[8u * p] == zero;
    let previous_slip = vec2<f64>(state[8u * p + 1u], state[8u * p + 2u]);
    let scale = state[8u * p + 3u];
    let pair = point_pair(p);
    let pn = point_count(p);
    var x = vec3<f64>(zero, zero, zero);
    for (var i = 0u; i < pn; i++) {
        x += pdata(p, i) * current(point_node(p, i));
    }
    // The closest candidate face (`Pair::closest`).
    var found = false;
    var nearest_reach = zero;
    var first = 0u;
    let c_from = candidates_from(p);
    let c_to = candidates_to(p);
    if pr(pair, 14u) != zero {
        for (var k = c_from; k < c_to; k++) {
            let f = candidate(k);
            let reach = box_distance(f, x);
            if (!found || reach < nearest_reach || (reach == nearest_reach && f < first)) && !own(p, f) {
                found = true;
                nearest_reach = reach;
                first = f;
            }
        }
    }
    if !found {
        write_open(p, zero);
        if !was_open {
            atomicOr(&flags[0], 1u);
        }
        return;
    }
    var best_d = zero;
    var best_interior = false;
    var best_f = first;
    var best_p = vec2<f64>(zero, zero);
    var have = false;
    for (var k = c_from; k <= c_to; k++) {
        // `first` before the others.
        var f = first;
        if k > c_from {
            f = candidate(k - 1u);
            if f == first {
                continue;
            }
            if have && best_interior && box_distance(f, x) > best_d {
                continue;
            }
            if own(p, f) {
                continue;
            }
        }
        let natural = project_natural(f, x);
        let at = place(f, natural);
        let distance = norm3(x - at.x);
        let interior = is_interior(f, natural);
        var better = !have;
        if have {
            better = (interior && !best_interior)
                || (interior == best_interior && (distance < best_d || (distance == best_d && f < best_f)));
        }
        if better {
            have = true;
            best_d = distance;
            best_interior = interior;
            best_f = f;
            best_p = natural;
        }
    }
    let f = best_f;
    // The geometry at the foot point (`ContactFace::at`); `sn` keeps its shape functions.
    let fp = place(f, best_p);
    var n = cross3(fp.t0, fp.t1);
    let length = norm3(n);
    let planar = face_planar(f) != 0u;
    if length > zero {
        n = n * (one / length);
    } else if planar {
        n = vec3<f64>(zero, one, zero);
    } else {
        n = vec3<f64>(zero, zero, one);
    }
    let geometric_gap = dot3(x - fp.x, n);
    let node_to_face = pr(pair, 0u) != zero;
    let typical = pr(pair, 11u);
    if !node_to_face {
        var r = x - fp.x;
        if planar {
            r.z = zero;
        }
        let lateral = norm3(r - geometric_gap * n);
        if lateral > reals[4] * typical && !inner_edge(p, f) {
            write_open(p, zero);
            if !was_open {
                atomicOr(&flags[0], 1u);
            }
            return;
        }
    }
    let initial_gap = pdata(p, 10u);
    var gap = geometric_gap;
    if pr(pair, 9u) != zero {
        gap = geometric_gap - initial_gap + pr(pair, 10u);
    }
    // The nodes of the spring: the point's with their weights, the face's with -N.
    var nodes = array<u32, 16>();
    var coef = array<f64, 16>();
    var m = 0u;
    for (var i = 0u; i < pn; i++) {
        nodes[m] = point_node(p, i);
        coef[m] = pdata(p, i);
        m++;
    }
    let fm = face_count(f);
    for (var i = 0u; i < fm; i++) {
        nodes[m] = face_node(f, i);
        coef[m] = -sn[i];
        m++;
    }
    var relative = vec3<f64>(zero, zero, zero);
    for (var a = 0u; a < m; a++) {
        let k = 3u * nodes[a];
        relative += coef[a] * vec3<f64>(u[k], u[k + 1u], u[k + 2u]);
    }
    let area = pdata(p, 8u);
    let cutoff = pdata(p, 9u);
    // The response (`Pair::respond`, not tied, no predictor, damped).
    let overclosure = -gap;
    let within = gap < cutoff;
    let open_face = !node_to_face && overclosure <= zero;
    let open_node = node_to_face && pr(pair, 12u) != zero && overclosure <= zero;
    if !within || open_face || open_node {
        write_open(p, gap);
        if !was_open {
            atomicOr(&flags[0], 1u);
        }
        return;
    }
    let law = u32(f32(pr(pair, 1u)));
    var pd = vec2<f64>(zero, zero);
    if law == 0u {
        // The rounded start of the linear relation in a dynamic step.
        let k = pr(pair, 2u);
        let delta = reals[3] * typical;
        if delta > zero {
            if overclosure <= zero {
                pd = vec2<f64>(zero, zero);
            } else if overclosure < delta {
                pd = vec2<f64>(f64(0.5) * k * overclosure * overclosure / delta, k * overclosure / delta);
            } else {
                pd = vec2<f64>(k * (overclosure - f64(0.5) * delta), k);
            }
        } else if node_to_face {
            pd = node_pressure(pair, overclosure);
        } else {
            pd = face_pressure(pair, overclosure);
        }
    } else if node_to_face {
        pd = node_pressure(pair, overclosure);
    } else {
        pd = face_pressure(pair, overclosure);
    }
    var pressure = scale * pd.x;
    let dp = scale * pd.y;
    // The tangential basis and the relative tangential motion.
    let t1 = normalized3(fp.t0 - dot3(fp.t0, n) * n);
    let t2 = cross3(n, t1);
    let edges = pr(pair, 13u) != zero;
    var tangential = vec2<f64>(dot3(relative, t1), dot3(relative, t2));
    if edges {
        tangential.y = zero;
    }
    var shear = vec2<f64>(zero, zero);
    var slip = previous_slip;
    var status = 1u;
    var tan00 = zero;
    var tan01 = zero;
    var tan11 = zero;
    let friction = pr(pair, 6u) != zero;
    if friction && (pressure > zero || (node_to_face && pressure < zero)) {
        let lambda = scale * pr(pair, 8u);
        let trial = lambda * (tangential - slip);
        let size = sqrt64(trial.x * trial.x + trial.y * trial.y);
        let limit = pr(pair, 7u) * abs64(pressure);
        if size <= limit || size == zero {
            shear = trial;
            tan00 = lambda;
            tan11 = lambda;
        } else {
            status = 2u;
            let dir = trial * (one / size);
            shear = limit * dir;
            slip = tangential - shear * (one / lambda);
            let a = lambda * limit / size;
            tan00 = a * (one - dir.x * dir.x);
            tan01 = -a * dir.x * dir.y;
            tan11 = a * (one - dir.y * dir.y);
        }
    } else {
        slip = tangential;
        if pressure <= zero {
            status = 0u;
        }
    }
    if edges {
        shear.y = zero;
        slip.y = zero;
        tan01 = zero;
        tan11 = zero;
    }
    if node_to_face && pressure <= zero && friction {
        status = 0u;
    }
    // The damping of a pressed spring.
    var q = zero;
    if pressure > zero && !was_open {
        var rate = zero;
        for (var a = 0u; a < m; a++) {
            let k = 3u * nodes[a];
            rate -= coef[a] * (v[k] * n.x + v[k + 1u] * n.y + v[k + 2u] * n.z);
        }
        let c = reals[2] * step.h * dp;
        q = max64(c * rate, -pressure);
        pressure += q;
    }
    let traction = -pressure * n + shear.x * t1 + shear.y * t2;
    // The 3x3 kernel `dp n nᵀ + Σ D_kl t_k t_lᵀ` for the Gershgorin rows.
    var kernel = array<vec3<f64>, 3>();
    for (var i = 0u; i < 3u; i++) {
        for (var j = 0u; j < 3u; j++) {
            kernel[i][j] = dp * n[i] * n[j] + tan00 * t1[i] * t1[j] + tan01 * (t1[i] * t2[j] + t2[i] * t1[j])
                + tan11 * t2[i] * t2[j];
        }
    }
    // Σ_bj |c_b K_ij| w_bj per row i.
    var sums = vec3<f64>(zero, zero, zero);
    for (var b = 0u; b < m; b++) {
        let k = 3u * nodes[b];
        let cb = abs64(coef[b]);
        for (var i = 0u; i < 3u; i++) {
            for (var j = 0u; j < 3u; j++) {
                sums[i] += cb * abs64(kernel[i][j]) * rows[cd.n + k + j];
            }
        }
    }
    for (var a = 0u; a < m; a++) {
        let s = slot_of(p, nodes[a]);
        if s == 0xffffffffu {
            continue;
        }
        let k = 3u * nodes[a];
        let ca = coef[a] * area;
        for (var d = 0u; d < 3u; d++) {
            let fe = ca * traction[d];
            let damp = -ca * q * n[d];
            slots[9u * s + d] += fe - damp;
            slots[9u * s + 3u + d] += damp;
            slots[9u * s + 6u + d] += abs64(ca) * sums[d] * rows[cd.n + k + d];
        }
    }
    state[8u * p] = f64(status);
    state[8u * p + 1u] = slip.x;
    state[8u * p + 2u] = slip.y;
    state[8u * p + 4u] = gap;
    state[8u * p + 5u] = pressure;
    state[8u * p + 6u] = f64(f);
    if was_open != (status == 0u) {
        atomicOr(&flags[0], 1u);
    }
}

fn gathered(wid: vec3<u32>, groups: vec3<u32>, li: u32) -> u32 {
    return (wid.y * groups.x + wid.x) * 64u + li;
}

// The rows of the springs per DOF.
@compute @workgroup_size(64)
fn contact_rows(
    @builtin(workgroup_id) wid: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let k = gathered(wid, groups, li);
    if k >= cd.gnodes {
        return;
    }
    let node = lists[cd.gnode_base + k];
    var sum = vec3<f64>(f64(0.0), f64(0.0), f64(0.0));
    for (var j = lists[cd.nslot_ptr_base + k]; j < lists[cd.nslot_ptr_base + k + 1u]; j++) {
        let s = lists[cd.nslot_base + j];
        sum += vec3<f64>(slots[9u * s + 6u], slots[9u * s + 7u], slots[9u * s + 8u]);
    }
    rows[3u * node] = sum.x;
    rows[3u * node + 1u] = sum.y;
    rows[3u * node + 2u] = sum.z;
}

// Springs too stiff for the time step (Gershgorin's bound of all springs on the masses of
// their nodes above `0.7/h²`) get their stiffness lowered, for good.
@compute @workgroup_size(64)
fn contact_cap(
    @builtin(workgroup_id) wid: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let p = gathered(wid, groups, li);
    if p >= cd.points {
        return;
    }
    let face = state[8u * p + 6u];
    if face < f64(0.0) {
        return;
    }
    let f = u32(f32(face));
    // The largest row over the nodes of the point and of its face.
    let pc = point_count(p);
    let count = pc + face_count(f);
    var largest = f64(0.0);
    for (var i = 0u; i < count; i++) {
        var node = 0u;
        if i < pc {
            node = point_node(p, i);
        } else {
            node = face_node(f, i - pc);
        }
        for (var d = 0u; d < 3u; d++) {
            let r = rows[3u * node + d];
            if r > largest {
                largest = r;
            }
        }
    }
    let limit = reals[1] / (step.h * step.h);
    if largest > limit {
        let factor = limit / largest;
        state[8u * p + 3u] *= factor;
        for (var s = lists[cd.slot_ptr_base + p]; s < lists[cd.slot_ptr_base + p + 1u]; s++) {
            for (var d = 0u; d < 6u; d++) {
                slots[9u * s + d] *= factor;
            }
        }
    }
}

// The forces of the springs per DOF: the spring into the internal forces, its damping
// into the viscous ones.
@compute @workgroup_size(64)
fn contact_gather(
    @builtin(workgroup_id) wid: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let k = gathered(wid, groups, li);
    if k >= cd.gnodes {
        return;
    }
    let node = lists[cd.gnode_base + k];
    var spring = vec3<f64>(f64(0.0), f64(0.0), f64(0.0));
    var damp = spring;
    for (var j = lists[cd.nslot_ptr_base + k]; j < lists[cd.nslot_ptr_base + k + 1u]; j++) {
        let s = lists[cd.nslot_base + j];
        spring += vec3<f64>(slots[9u * s], slots[9u * s + 1u], slots[9u * s + 2u]);
        damp += vec3<f64>(slots[9u * s + 3u], slots[9u * s + 4u], slots[9u * s + 5u]);
    }
    for (var d = 0u; d < 3u; d++) {
        fs[3u * node + d] += spring[d];
        fs[cd.n + 3u * node + d] += damp[d];
    }
}

// A contact node that moved farther than the search covers since the last search.
@compute @workgroup_size(64)
fn contact_check(
    @builtin(workgroup_id) wid: vec3<u32>,
    @builtin(num_workgroups) groups: vec3<u32>,
    @builtin(local_invocation_index) li: u32,
) {
    let k = gathered(wid, groups, li);
    if k >= cd.checks {
        return;
    }
    let node = lists[cd.check_base + k];
    let at = 3u * node;
    let d = vec3<f64>(u[at] - prepared[3u * k], u[at + 1u] - prepared[3u * k + 1u], u[at + 2u] - prepared[3u * k + 2u]);
    let moved = sqrt64(dot3(d, d));
    if !(moved < reals[0]) {
        atomicOr(&flags[1], 1u);
    }
}

// First in every time step: stops the run after the time step before when that one asks
// for a search or (from `events_from` on) opened or closed a contact.
@compute @workgroup_size(1)
fn contact_gate() {
    let index = u32(f32(step.index));
    if atomicLoad(&flags[2]) != 0u {
        return;
    }
    let changed = atomicLoad(&flags[0]);
    let search = atomicLoad(&flags[1]);
    atomicStore(&flags[4], changed);
    let event = changed != 0u && index >= 1u && index - 1u >= cd.events_from;
    if index >= 1u && (search != 0u || event) {
        atomicStore(&flags[2], index);
        var why = 0u;
        if search != 0u {
            why |= 1u;
        }
        if event {
            why |= 2u;
        }
        atomicStore(&flags[3], why);
        for (var w = 0u; w < cd.arg_words; w++) {
            args[w] = 0u;
        }
        return;
    }
    atomicStore(&flags[0], 0u);
    atomicStore(&flags[1], 0u);
}
