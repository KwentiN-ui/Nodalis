// The time step of the material point method on the graphics card: the same steps as the
// processor's (`solver.rs`), in single precision, with positions relative to node 0 of the
// grid and temperatures relative to a reference, the temperature of a particle carried as
// a compensated pair. No sum depends on the order the threads run in: the particles are
// sorted by the cell of their stencil and by their number within a cell, every node gathers
// what the particles of the 27 cells around it give (no atomic adds of floats), and the sums
// over particles and nodes are folded in a fixed order.
//
// A step: `control` (the time step, the poses of the walls, the dispatches of the step or
// none once the run waits for the processor), `key`, the scan of the cell counts, `scatter`
// and `cells` (the sort), `record` (what each particle spreads), `gather` (particles to
// grid), `grid` (gravity, walls, sides, the loads on them), `loads`, `conduct`, `g2p` (grid
// to particles, the material law, the motion) and `finish` (the sums of the step, the time,
// whether a frame or a history point is due).

const NONE: u32 = 0xffffffffu;
const WALL_REACH: f32 = 0.2;
const VOLUME_RELAX: f32 = 0.05;
const SUBCELL_CONDUCTION: f32 = 0.3;
// Nodes per tile and axis, tiles a workgroup of `grid` takes one after the other.
const TILE: u32 = 4u;
const GRID_TILES: u32 = 8u;
const SCAN: u32 = 256u;
// Values per load: force, moment, work, heat.
const LOAD: u32 = 8u;
// Values per sum of the particles: kinetic, potential, stored, dissipated, fastest, mass
// and energy that left, particles that left.
const SUMS: u32 = 8u;

// Why the run waits for the processor.
const FRAME: u32 = 1u;
const HISTORY: u32 = 2u;
const BROKEN: u32 = 4u;
const EMPTY: u32 = 8u;

override LEVEL: u32 = 0u;

struct Params {
    // Nodes per axis; w: nodes.
    n: vec4u,
    // Tiles per axis; w: tiles.
    tiles: vec4u,
    // Periodic axes; w: particle slots.
    periodic: vec4u,
    // The first node on or beyond the box per axis; w: walls.
    lo: vec4u,
    // The last node on or beyond the box per axis; w: whether the temperature is computed.
    hi: vec4u,
    // Node 0 in the model; w: the cell size.
    origin: vec4f,
    // The box relative to node 0; w: Courant number, wave speed.
    box_lo: vec4f,
    box_hi: vec4f,
    // Time step limits of viscosity and conduction, wall heat transfer, F-bar (0 or 1).
    limits: vec4f,
    // Side laws (0 open, 1 stick, 2 slip, 3 Coulomb) x min, x max, y min, y max; z min, z max.
    sides0: vec4u,
    sides1: vec4u,
    friction0: vec4f,
    friction1: vec4f,
    // Volume of a particle, the slots of the loads (walls, six sides, the sum), workgroups of
    // `g2p`, workgroups of `grid`.
    volume: f32,
    slots: u32,
    particle_groups: u32,
    grid_groups: u32,
    // In `partials`: the loads of the step, the sums of the particles of the step.
    offsets: vec4u,
    // The scan: levels, and per level its start in `scan` and its length.
    scan_levels: vec4u,
    scan_off: vec4u,
    scan_len: vec4u,
}

struct Control {
    running: u32,
    reason: u32,
    steps: u32,
    hit: u32,
    dt: f32,
    tl: f32,
    dt_max: f32,
    dt_min: f32,
    frame_at: f32,
    history: f32,
    base: f32,
    max_speed: f32,
    top_speed: f32,
    kinetic: f32,
    potential: f32,
    stored: f32,
    dissipated: vec2f,
    loss: vec2f,
    work: vec2f,
    out_mass: vec2f,
    out_energy: vec2f,
    samples: u32,
    gone: u32,
    pad0: u32,
    pad1: u32,
}

struct Particle {
    // Position relative to node 0; w: temperature (relative), high part.
    x: vec4f,
    // Velocity; w: temperature, low part.
    v: vec4f,
    // The rows of C; w: volume ratio, plastic shear, shear rate.
    c0: vec4f,
    c1: vec4f,
    c2: vec4f,
    // Elastic b: 00 11 22 01; then 12 02 and the Kirchhoff stress 12 02.
    b: vec4f,
    tau: vec4f,
    bt: vec4f,
    // Number, material | body << 16, left the model.
    info: vec4u,
}

struct Record {
    // The fraction of the stencil; w: mass.
    frac: vec4f,
    // The rows of the affine momentum with the stress folded in; w: mass × volume ratio,
    // heat capacity, heat.
    q0: vec4f,
    q1: vec4f,
    q2: vec4f,
    // The stress term: 00 11 22 01; 12 02, mass × conductivity, volume.
    s0: vec4f,
    s1: vec4f,
    // Momentum; weight.
    mv: vec4f,
    weight: vec4f,
}

struct Node {
    // Mass, momentum.
    a: vec4f,
    // Force of the stresses, mass × volume ratio.
    b: vec4f,
    // Weight, volume.
    c: vec4f,
    // Heat capacity, heat, mass × conductivity.
    d: vec4f,
}

struct Law {
    // Density, bulk, shear, kind (0 elastic, 1 Herschel-Bulkley, 2 fluid, 3 granular).
    a: vec4f,
    // Yield stress or viscosity, consistency, exponent (granular: static and limit
    // friction, inertial number I0), heat capacity per volume.
    b: vec4f,
    // Conductivity, mass of a particle, grain diameter.
    c: vec4f,
    // Start and length of the flow factor table in `tables`.
    d: vec4u,
}

struct Wall {
    // Lattice origin relative to node 0; w: spacing.
    origin: vec4f,
    // Lattice points per axis; w: start in `tables`.
    dims: vec4u,
    // Center of the motion at the start; w: law (1 stick, 2 slip, 3 Coulomb).
    center: vec4f,
    // Translation per unit of s; w: friction.
    velocity: vec4f,
    // Rotation per unit of s; w: temperature (relative).
    angular: vec4f,
    // Start and length of the amplitude in `tables`, whether it has a temperature.
    extra: vec4u,
}

struct Pose {
    r0: vec4f,
    r1: vec4f,
    r2: vec4f,
    // Center where it is; w: factor on the velocities.
    center: vec4f,
    lo: vec4f,
    hi: vec4f,
}

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read_write> ctl: Control;
@group(0) @binding(2) var<storage, read_write> particles: array<Particle>;
@group(0) @binding(3) var<storage, read_write> records: array<Record>;
@group(0) @binding(4) var<storage, read_write> keys: array<u32>;
@group(0) @binding(5) var<storage, read_write> counts: array<atomic<u32>>;
@group(0) @binding(6) var<storage, read_write> fill: array<atomic<u32>>;
@group(0) @binding(7) var<storage, read_write> scan: array<u32>;
@group(0) @binding(8) var<storage, read_write> order: array<u32>;
@group(0) @binding(9) var<storage, read_write> nodes: array<Node>;
@group(0) @binding(10) var<storage, read_write> values: array<vec4f>;
@group(0) @binding(11) var<storage, read_write> heat: array<vec2f>;
@group(0) @binding(12) var<storage, read> laws: array<Law>;
@group(0) @binding(13) var<storage, read> walls: array<Wall>;
@group(0) @binding(14) var<storage, read_write> poses: array<Pose>;
@group(0) @binding(15) var<storage, read> tables: array<f32>;
@group(0) @binding(16) var<storage, read> bodies: array<vec4f>;
@group(0) @binding(17) var<storage, read_write> partials: array<f32>;
@group(0) @binding(18) var<storage, read_write> loads: array<f32>;
// Only `control` binds these: the dispatches of a step, and the ones it copies.
@group(0) @binding(19) var<storage, read_write> args: array<u32>;
@group(0) @binding(20) var<storage, read> plan: array<u32>;

var<workgroup> shared_f: array<f32, 64>;
var<workgroup> shared_u: array<u32, 256>;

fn thread_index(wid: vec3u, groups: vec3u, lid: u32, size: u32) -> u32 {
    return (wid.y * groups.x + wid.x) * size + lid;
}

fn group_index(wid: vec3u, groups: vec3u) -> u32 {
    return wid.y * groups.x + wid.x;
}

fn node_index(i: vec3u) -> u32 {
    return i.x + params.n.x * (i.y + params.n.y * i.z);
}

fn wrap(a: u32, i: i32) -> u32 {
    let n = i32(params.n[a]);
    if params.periodic[a] != 0u {
        return u32(((i % n) + n) % n);
    }
    return u32(clamp(i, 0, n - 1));
}

fn h() -> f32 {
    return params.origin.w;
}

// The first node of the stencil of `x` (unwrapped) and the fraction.
fn stencil(x: vec3f) -> array<vec3f, 2> {
    let u = x / h();
    let base = floor(u - vec3f(0.5));
    return array<vec3f, 2>(base, u - base);
}

fn weights(f: f32) -> vec3f {
    return vec3f(
        0.5 * (1.5 - f) * (1.5 - f),
        0.75 - (f - 1.0) * (f - 1.0),
        0.5 * (f - 0.5) * (f - 0.5),
    );
}

fn cell_of(base: vec3f) -> u32 {
    let b = vec3i(base);
    return node_index(vec3u(wrap(0u, b.x), wrap(1u, b.y), wrap(2u, b.z)));
}

// a + x as a compensated pair.
fn add2(a: vec2f, x: f32) -> vec2f {
    let s = a.x + x;
    let bp = s - a.x;
    let e = (a.x - (s - bp)) + (x - bp);
    let lo = a.y + e;
    let hi = s + lo;
    return vec2f(hi, lo - (hi - s));
}

fn sym_matrix(d: vec3f, o: vec3f) -> mat3x3f {
    // d: 00 11 22; o: 01 12 02.
    return mat3x3f(
        vec3f(d.x, o.x, o.z),
        vec3f(o.x, d.y, o.y),
        vec3f(o.z, o.y, d.z),
    );
}

fn particle_c(p: Particle) -> mat3x3f {
    // Rows as stored; WGSL matrices take columns.
    return transpose(mat3x3f(p.c0.xyz, p.c1.xyz, p.c2.xyz));
}

fn particle_b(p: Particle) -> mat3x3f {
    return sym_matrix(p.b.xyz, vec3f(p.b.w, p.bt.x, p.bt.y));
}

fn particle_tau(p: Particle) -> mat3x3f {
    return sym_matrix(p.tau.xyz, vec3f(p.tau.w, p.bt.z, p.bt.w));
}

fn material(p: Particle) -> u32 {
    return p.info.y & 0xffffu;
}

fn body(p: Particle) -> u32 {
    return p.info.y >> 16u;
}

fn heat_on() -> bool {
    return params.hi.w != 0u;
}

// A table of (x, y) points in `tables`, linear in between and held beyond the ends; empty
// is 1.
fn table(start: u32, len: u32, x: f32) -> f32 {
    if len == 0u {
        return 1.0;
    }
    let first = vec2f(tables[start], tables[start + 1u]);
    let last = vec2f(tables[start + 2u * (len - 1u)], tables[start + 2u * (len - 1u) + 1u]);
    if x <= first.x {
        return first.y;
    }
    if x >= last.x {
        return last.y;
    }
    var k = 1u;
    loop {
        if k >= len - 1u || tables[start + 2u * k] > x {
            break;
        }
        k += 1u;
    }
    let p0 = vec2f(tables[start + 2u * (k - 1u)], tables[start + 2u * (k - 1u) + 1u]);
    let p1 = vec2f(tables[start + 2u * k], tables[start + 2u * k + 1u]);
    if p1.x > p0.x {
        return p0.y + (p1.y - p0.y) * (x - p0.x) / (p1.x - p0.x);
    }
    return p1.y;
}

// ---- Walls ----

struct Probe {
    phi: f32,
    normal: vec3f,
    velocity: vec3f,
    found: bool,
}

fn pose_rotation(p: Pose) -> mat3x3f {
    return transpose(mat3x3f(p.r0.xyz, p.r1.xyz, p.r2.xyz));
}

fn probe(w: u32, x: vec3f) -> Probe {
    var out: Probe;
    out.found = false;
    let wall = walls[w];
    let pose = poses[w];
    let r = pose_rotation(pose);
    let local = transpose(r) * (x - pose.center.xyz) + wall.center.xyz;
    let spacing = wall.origin.w;
    let u = (local - wall.origin.xyz) / spacing;
    let dims = wall.dims.xyz;
    if !(all(u >= vec3f(0.0)) && all(u < vec3f(dims - vec3u(1u)))) {
        return out;
    }
    let base = vec3u(floor(u));
    let f = u - vec3f(base);
    var value = 0.0;
    var gradient = vec3f(0.0);
    for (var k = 0u; k < 2u; k++) {
        for (var j = 0u; j < 2u; j++) {
            for (var i = 0u; i < 2u; i++) {
                let wx = select(1.0 - f.x, f.x, i == 1u);
                let wy = select(1.0 - f.y, f.y, j == 1u);
                let wz = select(1.0 - f.z, f.z, k == 1u);
                let dx = select(-1.0, 1.0, i == 1u);
                let dy = select(-1.0, 1.0, j == 1u);
                let dz = select(-1.0, 1.0, k == 1u);
                let at = wall.dims.w + base.x + i + dims.x * (base.y + j + dims.y * (base.z + k));
                let v = tables[at];
                value += wx * wy * wz * v;
                gradient += vec3f(dx * wy * wz, wx * dy * wz, wx * wy * dz) * v / spacing;
            }
        }
    }
    let g = length(gradient);
    var normal = vec3f(0.0);
    if g > 1e-12 {
        normal = r * (gradient / g);
    }
    out.phi = value;
    out.normal = normal;
    out.velocity = (wall.velocity.xyz + cross(wall.angular.xyz, x - pose.center.xyz)) * pose.center.w;
    out.found = true;
    return out;
}

// A velocity after a wall law (1 stick, 2 slip, 3 Coulomb).
fn wall_velocity(law: u32, friction: f32, v: vec3f, wall: vec3f, n: vec3f) -> vec3f {
    let rel = v - wall;
    let vn = dot(rel, n);
    var out = vec3f(0.0);
    if law == 1u || all(n == vec3f(0.0)) {
        out = vec3f(0.0);
    } else if vn >= 0.0 {
        out = rel;
    } else if law == 2u {
        out = rel - n * vn;
    } else {
        let tangent = rel - n * vn;
        let t = length(tangent);
        if t <= -friction * vn {
            out = vec3f(0.0);
        } else {
            out = tangent * (1.0 + friction * vn / t);
        }
    }
    return out + wall;
}

fn rotation(axis: vec3f) -> mat3x3f {
    let angle = length(axis);
    if angle < 1e-30 {
        return mat3x3f(vec3f(1.0, 0.0, 0.0), vec3f(0.0, 1.0, 0.0), vec3f(0.0, 0.0, 1.0));
    }
    let k = axis / angle;
    let s = sin(angle);
    let c = cos(angle);
    let t = 1.0 - c;
    // Rows, transposed into columns.
    return transpose(mat3x3f(
        vec3f(c + k.x * k.x * t, k.x * k.y * t - k.z * s, k.x * k.z * t + k.y * s),
        vec3f(k.y * k.x * t + k.z * s, c + k.y * k.y * t, k.y * k.z * t - k.x * s),
        vec3f(k.z * k.x * t - k.y * s, k.z * k.y * t + k.x * s, c + k.z * k.z * t),
    ));
}

// s(t) and ds/dt of a wall's amplitude.
fn motion(w: u32, t: f32) -> vec2f {
    let start = walls[w].extra.x;
    let len = walls[w].extra.y;
    if len == 0u {
        return vec2f(t, 1.0);
    }
    let first = vec2f(tables[start], tables[start + 1u]);
    let last = vec2f(tables[start + 2u * (len - 1u)], tables[start + 2u * (len - 1u) + 1u]);
    // The factor: of the segment that starts at t.
    var k = 0u;
    loop {
        if k >= len || tables[start + 2u * k] > t {
            break;
        }
        k += 1u;
    }
    var factor = 0.0;
    if k > 0u && k < len {
        let p0 = vec2f(tables[start + 2u * (k - 1u)], tables[start + 2u * (k - 1u) + 1u]);
        let p1 = vec2f(tables[start + 2u * k], tables[start + 2u * k + 1u]);
        if p1.x > p0.x {
            factor = (p1.y - p0.y) / (p1.x - p0.x);
        }
    }
    var s = 0.0;
    if t <= first.x {
        s = first.y;
    } else if t >= last.x {
        s = last.y;
    } else {
        let j = max(k, 1u);
        let p0 = vec2f(tables[start + 2u * (j - 1u)], tables[start + 2u * (j - 1u) + 1u]);
        let p1 = vec2f(tables[start + 2u * j], tables[start + 2u * j + 1u]);
        if p1.x > p0.x {
            s = p0.y + (p1.y - p0.y) * (t - p0.x) / (p1.x - p0.x);
        } else {
            s = p1.y;
        }
    }
    return vec2f(s, factor);
}

// ---- The step ----

// The time step, the poses at its end and the dispatches: none once the run waits.
@compute @workgroup_size(64)
fn control(@builtin(local_invocation_index) lid: u32) {
    let words = arrayLength(&plan);
    if ctl.running == 0u {
        for (var i = lid; i < words; i += 64u) {
            args[i] = 0u;
        }
        return;
    }
    let courant = params.box_lo.w;
    let wave = params.box_hi.w;
    var dt = courant * h() / (wave + ctl.max_speed);
    dt = min(dt, min(params.limits.x, params.limits.y));
    let remaining = ctl.frame_at - ctl.tl;
    var hit = 0u;
    if dt >= remaining {
        dt = remaining;
        hit = 1u;
    } else if 2.0 * dt > remaining {
        dt = 0.5 * remaining;
    }
    // No thread writes what another reads.
    if !(dt > 0.0) {
        for (var i = lid; i < words; i += 64u) {
            args[i] = 0u;
        }
        if lid == 0u {
            ctl.running = 0u;
            ctl.reason |= BROKEN;
        }
        return;
    }
    let t = ctl.base + ctl.tl + dt;
    for (var w = lid; w < params.lo.w; w += 64u) {
        let wall = walls[w];
        // Where it is at the end of the step, how fast it moves in the middle.
        let m = vec2f(motion(w, t).x, motion(w, t - 0.5 * dt).y);
        let r = rotation(wall.angular.xyz * m.x);
        let center = wall.center.xyz + wall.velocity.xyz * m.x;
        var pose: Pose;
        let rows = transpose(r);
        pose.r0 = vec4f(rows[0], 0.0);
        pose.r1 = vec4f(rows[1], 0.0);
        pose.r2 = vec4f(rows[2], 0.0);
        pose.center = vec4f(center, m.y);
        let lo = wall.origin.xyz;
        let hi = wall.origin.xyz + vec3f(wall.dims.xyz - vec3u(1u)) * wall.origin.w;
        var bl = vec3f(1e30);
        var bh = vec3f(-1e30);
        for (var corner = 0u; corner < 8u; corner++) {
            let p = select(lo, hi, vec3<bool>((corner & 1u) != 0u, (corner & 2u) != 0u, (corner & 4u) != 0u));
            let q = r * (p - wall.center.xyz) + center;
            bl = min(bl, q);
            bh = max(bh, q);
        }
        pose.lo = vec4f(bl, 0.0);
        pose.hi = vec4f(bh, 0.0);
        poses[w] = pose;
    }
    for (var i = lid; i < words; i += 64u) {
        args[i] = plan[i];
    }
    if lid == 0u {
        ctl.dt = dt;
        ctl.hit = hit;
        ctl.dt_max = max(ctl.dt_max, dt);
        ctl.dt_min = min(ctl.dt_min, dt);
    }
}

// The cell of each particle's stencil, counted.
@compute @workgroup_size(64)
fn key(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) groups: vec3u,
       @builtin(local_invocation_index) lid: u32) {
    let p = thread_index(wid, groups, lid, 64u);
    if p >= params.periodic.w {
        return;
    }
    let part = particles[p];
    if part.info.z != 0u {
        keys[p] = NONE;
        return;
    }
    let s = stencil(part.x.xyz);
    let c = cell_of(s[0]);
    keys[p] = c;
    atomicAdd(&counts[c], 1u);
}

// One level of the exclusive scan of the counts: within groups of 256, the totals of the
// groups to the next level.
@compute @workgroup_size(256)
fn scan_local(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) groups: vec3u,
              @builtin(local_invocation_index) lid: u32) {
    let g = group_index(wid, groups);
    let i = g * SCAN + lid;
    let len = params.scan_len[LEVEL];
    let off = params.scan_off[LEVEL];
    var value = 0u;
    if i < len {
        if LEVEL == 0u {
            value = atomicLoad(&counts[i]);
        } else {
            value = scan[off + i];
        }
    }
    shared_u[lid] = value;
    workgroupBarrier();
    for (var step = 1u; step < SCAN; step <<= 1u) {
        var add = 0u;
        if lid >= step {
            add = shared_u[lid - step];
        }
        workgroupBarrier();
        shared_u[lid] += add;
        workgroupBarrier();
    }
    if i < len {
        scan[off + i] = shared_u[lid] - value;
    }
    if lid == SCAN - 1u && LEVEL + 1u < params.scan_levels.x {
        scan[params.scan_off[LEVEL + 1u] + g] = shared_u[lid];
    }
}

// Adds the scanned totals of the next level to a level.
@compute @workgroup_size(256)
fn scan_add(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) groups: vec3u,
            @builtin(local_invocation_index) lid: u32) {
    let g = group_index(wid, groups);
    let i = g * SCAN + lid;
    if i < params.scan_len[LEVEL] {
        scan[params.scan_off[LEVEL] + i] += scan[params.scan_off[LEVEL + 1u] + g];
    }
}

@compute @workgroup_size(64)
fn scatter(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) groups: vec3u,
           @builtin(local_invocation_index) lid: u32) {
    let p = thread_index(wid, groups, lid, 64u);
    if p >= params.periodic.w {
        return;
    }
    let k = keys[p];
    if k == NONE {
        return;
    }
    let slot = scan[k] + atomicAdd(&fill[k], 1u);
    order[slot] = p;
}

// The particles of a cell in the order of their numbers; the counts back to zero.
@compute @workgroup_size(64)
fn cells(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) groups: vec3u,
         @builtin(local_invocation_index) lid: u32) {
    let c = thread_index(wid, groups, lid, 64u);
    if c >= params.n.w {
        return;
    }
    let start = scan[c];
    let end = scan[c + 1u];
    for (var i = start + 1u; i < end; i++) {
        let item = order[i];
        var j = i;
        loop {
            if j <= start || order[j - 1u] <= item {
                break;
            }
            order[j] = order[j - 1u];
            j -= 1u;
        }
        order[j] = item;
    }
    atomicStore(&counts[c], 0u);
    atomicStore(&fill[c], 0u);
}

fn total() -> u32 {
    return scan[params.n.w];
}

// What each particle spreads, in the order of the cells.
@compute @workgroup_size(64)
fn record(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) groups: vec3u,
          @builtin(local_invocation_index) lid: u32) {
    let s = thread_index(wid, groups, lid, 64u);
    if s >= total() {
        return;
    }
    let p = particles[order[s]];
    let law = laws[material(p)];
    let mass = law.c.y;
    let volume = params.volume;
    let dt = ctl.dt;
    let inv = 4.0 / (h() * h());
    let tau = particle_tau(p);
    let q = particle_c(p) * mass - tau * (dt * volume * inv);
    let stress = tau * (-volume * inv);
    let j = p.c0.w;
    var hc = 0.0;
    var conduction = 0.0;
    if heat_on() {
        hc = law.b.w * volume;
        conduction = mass * law.c.x;
    }
    let qr = transpose(q);
    var r: Record;
    r.frac = vec4f(stencil(p.x.xyz)[1], mass);
    r.q0 = vec4f(qr[0], mass * j);
    r.q1 = vec4f(qr[1], hc);
    r.q2 = vec4f(qr[2], hc * (p.x.w + p.v.w));
    r.s0 = vec4f(stress[0][0], stress[1][1], stress[2][2], stress[1][0]);
    r.s1 = vec4f(stress[2][1], stress[2][0], conduction, volume * j);
    r.mv = vec4f(p.v.xyz * mass, 0.0);
    r.weight = vec4f(bodies[body(p)].xyz * mass, 0.0);
    records[s] = r;
}

// The node of a thread of a tile workgroup, and whether it is in the grid.
fn tile_node(tile: u32, lid: u32) -> vec4u {
    let t = vec3u(
        tile % params.tiles.x,
        (tile / params.tiles.x) % params.tiles.y,
        tile / (params.tiles.x * params.tiles.y),
    );
    let l = vec3u(lid % TILE, (lid / TILE) % TILE, lid / (TILE * TILE));
    let i = t * TILE + l;
    let inside = tile < params.tiles.w && all(i < params.n.xyz);
    return vec4u(i, select(0u, 1u, inside));
}

// Particles to grid: every node sums what the particles of the 27 cells around it give.
@compute @workgroup_size(64)
fn gather(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) groups: vec3u,
          @builtin(local_invocation_index) lid: u32) {
    let at = tile_node(group_index(wid, groups), lid);
    if at.w == 0u {
        return;
    }
    let i = vec3i(at.xyz);
    let hh = h();
    var node: Node;
    node.a = vec4f(0.0);
    node.b = vec4f(0.0);
    node.c = vec4f(0.0);
    node.d = vec4f(0.0);
    for (var oz = 0; oz < 3; oz++) {
        let cz = i.z - 2 + oz;
        if params.periodic.z == 0u && (cz < 0 || cz >= i32(params.n.z)) {
            continue;
        }
        for (var oy = 0; oy < 3; oy++) {
            let cy = i.y - 2 + oy;
            if params.periodic.y == 0u && (cy < 0 || cy >= i32(params.n.y)) {
                continue;
            }
            for (var ox = 0; ox < 3; ox++) {
                let cx = i.x - 2 + ox;
                if params.periodic.x == 0u && (cx < 0 || cx >= i32(params.n.x)) {
                    continue;
                }
                let c = node_index(vec3u(wrap(0u, cx), wrap(1u, cy), wrap(2u, cz)));
                let sx = f32(2 - ox);
                let sy = f32(2 - oy);
                let sz = f32(2 - oz);
                let end = scan[c + 1u];
                for (var s = scan[c]; s < end; s++) {
                    let r = records[s];
                    let f = r.frac.xyz;
                    let wx = weights(f.x)[2 - ox];
                    let wy = weights(f.y)[2 - oy];
                    let wz = weights(f.z)[2 - oz];
                    let w = wx * wy * wz;
                    let d = (vec3f(sx, sy, sz) - f) * hh;
                    let qd = vec3f(dot(r.q0.xyz, d), dot(r.q1.xyz, d), dot(r.q2.xyz, d));
                    let fd = vec3f(
                        r.s0.x * d.x + r.s0.w * d.y + r.s1.y * d.z,
                        r.s0.w * d.x + r.s0.y * d.y + r.s1.x * d.z,
                        r.s1.y * d.x + r.s1.x * d.y + r.s0.z * d.z,
                    );
                    node.a += w * vec4f(r.frac.w, r.mv.xyz + qd);
                    node.b += w * vec4f(fd, r.q0.w);
                    node.c += w * vec4f(r.weight.xyz, r.s1.w);
                    node.d += w * vec4f(r.q1.w, r.q2.w, r.s1.z, 0.0);
                }
            }
        }
    }
    nodes[node_index(at.xyz)] = node;
}

fn side_law(side: u32) -> u32 {
    if side < 4u {
        return params.sides0[side];
    }
    return params.sides1[side - 4u];
}

fn side_friction(side: u32) -> f32 {
    if side < 4u {
        return params.friction0[side];
    }
    return params.friction1[side - 4u];
}

// The force a node exerts on a wall, its moment about the end of `arm`, the work the wall
// at velocity `wall` does on it.
fn record_load(load: ptr<function, array<f32, 8>>, mass: f32, before: vec3f, after: vec3f,
               dt: f32, arm: vec3f, wall: vec3f) {
    let force = (before - after) * (mass / dt);
    let moment = cross(arm, force);
    (*load)[0] += force.x;
    (*load)[1] += force.y;
    (*load)[2] += force.z;
    (*load)[3] += moment.x;
    (*load)[4] += moment.y;
    (*load)[5] += moment.z;
    (*load)[6] -= dot(force, wall) * dt;
}

// Sums (or the largest of) `value` over the workgroup of 64, for every thread.
fn fold64(lid: u32, value: f32, largest: bool) -> f32 {
    shared_f[lid] = value;
    workgroupBarrier();
    for (var step = 32u; step > 0u; step >>= 1u) {
        if lid < step {
            if largest {
                shared_f[lid] = max(shared_f[lid], shared_f[lid + step]);
            } else {
                shared_f[lid] += shared_f[lid + step];
            }
        }
        workgroupBarrier();
    }
    let out = shared_f[0];
    workgroupBarrier();
    return out;
}

// What a node of the grid gives its wall and the sides it is beyond.
struct NodeLoads {
    wall_slot: u32,
    wall_load: array<f32, 8>,
    side_slot: vec3u,
    side_load: array<array<f32, 8>, 3>,
    loss: f32,
    touched: bool,
}

fn grid_node(at: vec4u) -> NodeLoads {
    var out: NodeLoads;
    out.wall_slot = NONE;
    out.side_slot = vec3u(NONE);
    out.loss = 0.0;
    out.touched = false;
    for (var k = 0u; k < 8u; k++) {
        out.wall_load[k] = 0.0;
        for (var a = 0u; a < 3u; a++) {
            out.side_load[a][k] = 0.0;
        }
    }
    if at.w == 0u {
        return out;
    }
    let walls_n = params.lo.w;
    let dt = ctl.dt;
    let hh = h();
    let index = node_index(at.xyz);
    let node = nodes[index];
    var q = 0.0;
    let mass = node.a.x;
    if mass <= 0.0 {
        values[index] = vec4f(0.0);
        if heat_on() {
            heat[index].x = 0.0;
        }
        return out;
    }
    let before = (node.a.yzw + node.c.xyz * dt) / mass;
    var after = before;
    let x = vec3f(at.xyz) * hh;
    // The wall the node is deepest in.
    var owner = NONE;
    var deepest = 0.0;
    var owner_normal = vec3f(0.0);
    var owner_velocity = vec3f(0.0);
    for (var w = 0u; w < walls_n; w++) {
        let pose = poses[w];
        if any(x < pose.lo.xyz) || any(x > pose.hi.xyz) {
            continue;
        }
        let pr = probe(w, x);
        if pr.found && pr.phi <= WALL_REACH * hh && (owner == NONE || pr.phi < deepest) {
            deepest = pr.phi;
            owner = w;
            owner_normal = pr.normal;
            owner_velocity = pr.velocity;
        }
    }
    if owner != NONE {
        let wall = walls[owner];
        after = wall_velocity(u32(wall.center.w), wall.velocity.w, after, owner_velocity, owner_normal);
        out.wall_slot = owner;
        var l = out.wall_load;
        record_load(&l, mass, before, after, dt, x - poses[owner].center.xyz, owner_velocity);
        // Heat transfer with a wall that has a temperature, never past it.
        if heat_on() && wall.extra.z != 0u && node.d.x > 0.0 {
            let gap = wall.angular.w - node.d.y / node.d.x;
            var flow = params.limits.z * hh * hh * gap * dt;
            let most = 0.5 * node.d.x * abs(gap);
            flow = clamp(flow, -most, most);
            q += flow;
            l[7] += flow;
        }
        out.wall_load = l;
    }
    let xw = x + params.origin.xyz;
    for (var a = 0u; a < 3u; a++) {
        if params.periodic[a] != 0u {
            continue;
        }
        for (var e = 0u; e < 2u; e++) {
            let side = 2u * a + e;
            var beyond = at[a] <= params.lo[a];
            if e == 1u {
                beyond = at[a] >= params.hi[a];
            }
            let law = side_law(side);
            if !beyond || law == 0u {
                continue;
            }
            var n = vec3f(0.0);
            n[a] = select(-1.0, 1.0, e == 0u);
            let was = after;
            after = wall_velocity(law, side_friction(side), after, vec3f(0.0), n);
            out.side_slot[a] = walls_n + side;
            var l = out.side_load[a];
            record_load(&l, mass, was, after, dt, xw, vec3f(0.0));
            out.side_load[a] = l;
        }
    }
    // What the walls took: the kinetic energy they removed, less the work the forces on
    // the node would have done with the velocity removed.
    if any(after != before) {
        let pull = node.b.xyz + node.c.xyz;
        let own = before - pull * (dt / mass);
        let taken = 0.5 * mass * (dot(own, own) - dot(after, after)) + dt * dot(pull, after);
        out.loss = taken;
        out.touched = true;
        if heat_on() {
            q += max(taken, 0.0);
        }
    }
    values[index] = vec4f(after, node.b.w / mass);
    if heat_on() {
        heat[index].x = q;
    }
    return out;
}

var<workgroup> used: atomic<u32>;
var<workgroup> used_copy: u32;
var<workgroup> acc: array<f32, 256>;

// The grid: gravity, the walls and the sides of the box, the heat the walls give. A
// workgroup takes `GRID_TILES` tiles one after the other; the loads on the walls are summed
// over them per slot in a fixed order.
@compute @workgroup_size(64)
fn grid(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) groups: vec3u,
        @builtin(local_invocation_index) lid: u32) {
    let g = group_index(wid, groups);
    let slots = params.slots;
    for (var i = lid; i < slots * LOAD; i += 64u) {
        acc[i] = 0.0;
    }
    for (var tile_k = 0u; tile_k < GRID_TILES; tile_k++) {
        if lid == 0u {
            atomicStore(&used, 0u);
        }
        workgroupBarrier();
        let r = grid_node(tile_node(g * GRID_TILES + tile_k, lid));
        var bits = 0u;
        if r.wall_slot != NONE {
            bits |= 1u << r.wall_slot;
        }
        for (var a = 0u; a < 3u; a++) {
            if r.side_slot[a] != NONE {
                bits |= 1u << r.side_slot[a];
            }
        }
        if r.touched {
            bits |= 1u << (slots - 1u);
        }
        if bits != 0u {
            atomicOr(&used, bits);
        }
        workgroupBarrier();
        if lid == 0u {
            used_copy = atomicLoad(&used);
        }
        let mask = workgroupUniformLoad(&used_copy);
        for (var slot = 0u; slot < slots; slot++) {
            if (mask & (1u << slot)) == 0u {
                continue;
            }
            for (var k = 0u; k < LOAD; k++) {
                var value = 0.0;
                if slot == r.wall_slot {
                    value += r.wall_load[k];
                }
                for (var a = 0u; a < 3u; a++) {
                    if slot == r.side_slot[a] {
                        value += r.side_load[a][k];
                    }
                }
                if slot == slots - 1u && k == 0u {
                    value += r.loss;
                }
                let sum = fold64(lid, value, false);
                if lid == 0u {
                    acc[slot * LOAD + k] += sum;
                }
            }
        }
    }
    workgroupBarrier();
    for (var i = lid; i < slots * LOAD; i += 64u) {
        partials[g * slots * LOAD + i] = acc[i];
    }
}

// The loads of a step: one workgroup per slot sums its rows of the `grid` groups.
@compute @workgroup_size(64)
fn reduce_loads(@builtin(workgroup_id) wid: vec3u, @builtin(local_invocation_index) lid: u32) {
    let slot = wid.x;
    let slots = params.slots;
    for (var k = 0u; k < LOAD; k++) {
        var value = 0.0;
        for (var g = lid; g < params.grid_groups; g += 64u) {
            value += partials[(g * slots + slot) * LOAD + k];
        }
        let sum = fold64(lid, value, false);
        if lid == 0u {
            partials[params.offsets.x + slot * LOAD + k] = sum;
        }
    }
}

// The state of a node for conduction: temperature and conductivity times filling; w: 0
// without.
fn conduction_state(index: u32) -> vec3f {
    let n = nodes[index];
    if n.d.x <= 0.0 || n.a.x <= 0.0 {
        return vec3f(0.0);
    }
    let hh = h();
    let filling = min(n.c.w / (hh * hh * hh), 1.0);
    return vec3f(n.d.y / n.d.x, n.d.z / n.a.x * filling, 1.0);
}

// The change of the temperature of every node: conduction between neighbours and what the
// walls gave.
@compute @workgroup_size(64)
fn conduct(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) groups: vec3u,
           @builtin(local_invocation_index) lid: u32) {
    let at = tile_node(group_index(wid, groups), lid);
    if at.w == 0u {
        return;
    }
    let index = node_index(at.xyz);
    let me = conduction_state(index);
    if me.z == 0.0 {
        heat[index].y = 0.0;
        return;
    }
    let hh = h();
    var flow = 0.0;
    for (var a = 0u; a < 3u; a++) {
        for (var e = 0; e < 2; e++) {
            let next = i32(at[a]) + select(-1, 1, e == 1);
            if params.periodic[a] == 0u && (next < 0 || next >= i32(params.n[a])) {
                continue;
            }
            var j = at.xyz;
            j[a] = wrap(a, next);
            let other = conduction_state(node_index(j));
            if other.z != 0.0 {
                flow += min(me.y, other.y) * hh * (other.x - me.x);
            }
        }
    }
    heat[index].y = (flow * ctl.dt + heat[index].x) / nodes[index].d.x;
}

// ---- The material law ----

struct Update {
    tau: mat3x3f,
    b: mat3x3f,
    j: f32,
    plastic: f32,
    dissipated: f32,
    stored: f32,
    shear_rate: f32,
}

fn contract(a: mat3x3f, b: mat3x3f) -> f32 {
    return dot(a[0], b[0]) + dot(a[1], b[1]) + dot(a[2], b[2]);
}

fn deviator(a: mat3x3f) -> mat3x3f {
    let mean = (a[0][0] + a[1][1] + a[2][2]) / 3.0;
    var d = a;
    d[0][0] -= mean;
    d[1][1] -= mean;
    d[2][2] -= mean;
    return d;
}

// One Jacobi rotation of the symmetric `a` in the plane (p, q), `v` collecting them.
fn jacobi(a: ptr<function, array<array<f32, 3>, 3>>, v: ptr<function, array<array<f32, 3>, 3>>,
          p: u32, q: u32) {
    let apq = (*a)[p][q];
    if abs(apq) <= 1e-30 {
        return;
    }
    let theta = ((*a)[q][q] - (*a)[p][p]) / (2.0 * apq);
    var t = sign(theta) / (abs(theta) + sqrt(theta * theta + 1.0));
    if theta == 0.0 {
        t = 1.0;
    }
    let c = 1.0 / sqrt(t * t + 1.0);
    let s = t * c;
    for (var k = 0u; k < 3u; k++) {
        let akp = (*a)[k][p];
        let akq = (*a)[k][q];
        (*a)[k][p] = c * akp - s * akq;
        (*a)[k][q] = s * akp + c * akq;
    }
    for (var k = 0u; k < 3u; k++) {
        let apk = (*a)[p][k];
        let aqk = (*a)[q][k];
        (*a)[p][k] = c * apk - s * aqk;
        (*a)[q][k] = s * apk + c * aqk;
    }
    for (var k = 0u; k < 3u; k++) {
        let vkp = (*v)[k][p];
        let vkq = (*v)[k][q];
        (*v)[k][p] = c * vkp - s * vkq;
        (*v)[k][q] = s * vkp + c * vkq;
    }
}

struct Eigen {
    values: vec3f,
    // Column a is the vector of value a.
    vectors: mat3x3f,
}

fn eigen_symmetric(m: mat3x3f) -> Eigen {
    var a: array<array<f32, 3>, 3>;
    var v: array<array<f32, 3>, 3>;
    for (var r = 0u; r < 3u; r++) {
        for (var c = 0u; c < 3u; c++) {
            a[r][c] = m[c][r];
            v[r][c] = select(0.0, 1.0, r == c);
        }
    }
    for (var sweep = 0; sweep < 12; sweep++) {
        let off = abs(a[0][1]) + abs(a[0][2]) + abs(a[1][2]);
        let scale = abs(a[0][0]) + abs(a[1][1]) + abs(a[2][2]);
        if off <= 1e-9 * max(scale, 1e-30) {
            break;
        }
        jacobi(&a, &v, 0u, 1u);
        jacobi(&a, &v, 0u, 2u);
        jacobi(&a, &v, 1u, 2u);
    }
    var out: Eigen;
    out.values = vec3f(a[0][0], a[1][1], a[2][2]);
    // v[r][c]: row r, column c; the vector of value c is column c.
    out.vectors = mat3x3f(
        vec3f(v[0][0], v[1][0], v[2][0]),
        vec3f(v[0][1], v[1][1], v[2][1]),
        vec3f(v[0][2], v[1][2], v[2][2]),
    );
    return out;
}

fn plastic_increment(trial: f32, yield_stress: f32, g: f32, k: f32, n: f32, dt: f32) -> f32 {
    let over = trial - yield_stress;
    if k <= 0.0 {
        return over / g;
    }
    if abs(n - 1.0) < 1e-6 {
        return over / (g + k / dt);
    }
    var lo = 0.0;
    var hi = over / g;
    var x = 0.5 * hi;
    for (var i = 0; i < 60; i++) {
        let r = over - g * x - k * pow(max(x / dt, 1e-30), n);
        if r > 0.0 {
            lo = x;
        } else {
            hi = x;
        }
        let slope = g + k * n / dt * pow(max(x / dt, 1e-30), n - 1.0);
        var next = x + r / slope;
        if !(next > lo && next < hi) {
            next = 0.5 * (lo + hi);
        }
        if abs(next - x) <= 1e-7 * max(hi, 1e-30) {
            return next;
        }
        x = next;
    }
    return x;
}

// The friction of the mu(I) rheology at the shear rate `rate` and its derivative;
// `per_rate` is d sqrt(rho/p).
fn inertial_friction(law: Law, per_rate: f32, rate: f32) -> vec2f {
    let rise = law.b.y - law.b.x;
    if rise <= 0.0 || law.b.z <= 0.0 {
        return vec2f(law.b.x, 0.0);
    }
    let i = rate * per_rate;
    let s = law.b.z + i;
    return vec2f(law.b.x + rise * i / s, rise * law.b.z / (s * s) * per_rate);
}

// The plastic shear increment with trial - G x = p mu(I(x/dt)) (as in law.rs).
fn granular_increment(law: Law, trial: f32, pressure: f32, g: f32, dt: f32) -> f32 {
    if pressure <= 0.0 {
        return trial / g;
    }
    let per_rate = law.c.z * sqrt(law.a.x / pressure);
    let top = (trial - pressure * law.b.x) / g;
    var lo = 0.0;
    var hi = top;
    var x = hi;
    for (var i = 0; i < 60; i++) {
        let mu = inertial_friction(law, per_rate, x / dt);
        let r = trial - g * x - pressure * mu.x;
        if r > 0.0 {
            lo = x;
        } else {
            hi = x;
        }
        let slope = g + pressure * mu.y / dt;
        var next = x + r / slope;
        if !(next > lo && next < hi) {
            next = 0.5 * (lo + hi);
        }
        if abs(next - x) <= 1e-7 * max(top, 1e-30) {
            return next;
        }
        x = next;
    }
    return x;
}

fn law_update(law: Law, b_in: mat3x3f, j_state: f32, plastic_in: f32, f: mat3x3f, j: f32,
              gradient: mat3x3f, dt: f32, factor: f32) -> Update {
    var out: Update;
    let own = j_state * determinant(f);
    let d = 0.5 * (gradient + transpose(gradient));
    let dd = deviator(d);
    out.shear_rate = sqrt(2.0 * contract(dd, dd));
    out.plastic = plastic_in;
    let bulk = law.a.y;
    let g = law.a.z;
    let kind = u32(law.a.w);
    if kind == 2u {
        let viscosity = law.b.x * factor;
        let jt = min(j, 1.0);
        out.j = min(own, 1.0);
        let pressure_tau = bulk * log(jt);
        let identity = mat3x3f(vec3f(1.0, 0.0, 0.0), vec3f(0.0, 1.0, 0.0), vec3f(0.0, 0.0, 1.0));
        out.tau = dd * (2.0 * viscosity * jt) + identity * pressure_tau;
        out.dissipated = 2.0 * viscosity * jt * contract(dd, dd) * dt;
        out.stored = 0.5 * bulk * log(jt) * log(jt);
        out.b = identity;
        return out;
    }
    // Grains carry no tension: the volume of their stress stays at most the initial one.
    var js = j;
    var own_s = own;
    if kind == 3u {
        js = min(j, 1.0);
        own_s = min(own, 1.0);
    }
    var b = f * b_in * transpose(f);
    let det_b = determinant(b);
    if det_b > 0.0 {
        b = b * pow(js * js / det_b, 1.0 / 3.0);
    }
    let e = eigen_symmetric(b);
    let strain = 0.5 * log(max(e.values, vec3f(1e-30)));
    let volumetric = strain.x + strain.y + strain.z;
    var dev = strain - vec3f(volumetric / 3.0);
    let dev_norm = length(dev);
    out.dissipated = 0.0;
    if kind == 1u {
        let yield_stress = law.b.x * factor;
        let consistency = law.b.y * factor;
        let trial = sqrt(2.0) * g * dev_norm;
        if trial > yield_stress && g > 0.0 {
            let dgamma = plastic_increment(trial, yield_stress, g, consistency, law.b.z, dt);
            let reduced = trial - g * dgamma;
            dev = dev * (reduced / trial);
            out.plastic += dgamma;
            out.dissipated = reduced * dgamma;
        }
    }
    if kind == 3u {
        let trial = sqrt(2.0) * g * dev_norm;
        let pressure = -bulk * volumetric;
        if g > 0.0 && trial > law.b.x * pressure {
            let dgamma = granular_increment(law, trial, pressure, g, dt);
            let reduced = trial - g * dgamma;
            if trial > 0.0 {
                dev = dev * (reduced / trial);
            }
            out.plastic += dgamma;
            out.dissipated = reduced * dgamma;
        }
    }
    let total = dev + vec3f(volumetric / 3.0);
    let tau_principal = 2.0 * g * dev + vec3f(bulk * volumetric);
    let e2 = exp(2.0 * total);
    var tau = mat3x3f(vec3f(0.0), vec3f(0.0), vec3f(0.0));
    var bn = mat3x3f(vec3f(0.0), vec3f(0.0), vec3f(0.0));
    for (var a = 0u; a < 3u; a++) {
        let va = e.vectors[a];
        let outer = mat3x3f(va * va.x, va * va.y, va * va.z);
        tau += outer * tau_principal[a];
        bn += outer * e2[a];
    }
    out.tau = tau;
    out.b = bn;
    out.j = own_s;
    out.stored = g * dot(dev, dev) + 0.5 * bulk * volumetric * volumetric;
    return out;
}

// ---- Grid to particles ----

@compute @workgroup_size(64)
fn g2p(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) groups: vec3u,
       @builtin(local_invocation_index) lid: u32) {
    let s = thread_index(wid, groups, lid, 64u);
    let g = group_index(wid, groups);
    var sums = array<f32, 8>(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    if s < total() {
        let pi = order[s];
        var p = particles[pi];
        let law = laws[material(p)];
        let mass = law.c.y;
        let volume = params.volume;
        let dt = ctl.dt;
        let hh = h();
        let st = stencil(p.x.xyz);
        let base = vec3i(st[0]);
        let frac = st[1];
        let wx = weights(frac.x);
        let wy = weights(frac.y);
        let wz = weights(frac.z);
        var v = vec3f(0.0);
        var bm = mat3x3f(vec3f(0.0), vec3f(0.0), vec3f(0.0));
        var jbar = 0.0;
        var warmer = 0.0;
        var around = vec2f(0.0);
        let heat_on = heat_on();
        for (var sk = 0; sk < 3; sk++) {
            let gz = wrap(2u, base.z + sk);
            for (var sj = 0; sj < 3; sj++) {
                let gy = wrap(1u, base.y + sj);
                for (var si = 0; si < 3; si++) {
                    let gx = wrap(0u, base.x + si);
                    let weight = wx[si] * wy[sj] * wz[sk];
                    let at = node_index(vec3u(gx, gy, gz));
                    let node = values[at];
                    let dn = (vec3f(f32(si), f32(sj), f32(sk)) - frac) * hh;
                    let wv = node.xyz * weight;
                    v += wv;
                    bm += mat3x3f(wv * dn.x, wv * dn.y, wv * dn.z);
                    jbar += weight * node.w;
                    if heat_on {
                        let dtn = heat[at].y;
                        warmer += weight * dtn;
                        let n = nodes[at].d;
                        if n.x > 0.0 {
                            around += vec2f(weight * (n.y / n.x + dtn), weight);
                        }
                    }
                }
            }
        }
        let c = bm * (4.0 / (hh * hh));
        let identity = mat3x3f(vec3f(1.0, 0.0, 0.0), vec3f(0.0, 1.0, 0.0), vec3f(0.0, 0.0, 1.0));
        let f = identity + c * dt;
        let fbar = params.limits.w != 0.0;
        var source = p.c0.w;
        if fbar && jbar > 0.0 {
            source = jbar;
        }
        let j_new = source * determinant(f);
        var temperature = vec2f(p.x.w, p.v.w);
        let factor = table(law.d.x, law.d.y, temperature.x + temperature.y);
        let up = law_update(law, particle_b(p), p.c0.w, p.c1.w, f, j_new, c, dt, factor);
        let hc = law.b.w * volume;
        if heat_on && hc > 0.0 {
            temperature = add2(temperature, warmer);
            if around.y > 0.0 {
                let alpha = law.c.x * volume / hc;
                let pull = min(SUBCELL_CONDUCTION * alpha * dt / (hh * hh), 1.0);
                temperature = add2(temperature, pull * (around.x / around.y - (temperature.x + temperature.y)));
            }
            temperature = add2(temperature, up.dissipated * volume / hc);
        }
        var j_own = up.j;
        if fbar && jbar > 0.0 {
            j_own = j_own * exp(VOLUME_RELAX * log(j_new / max(j_own, 1e-30)));
        }
        var x = p.x.xyz + v * dt;
        for (var w = 0u; w < params.lo.w; w++) {
            let pr = probe(w, x);
            if pr.found && pr.phi < 0.0 {
                x -= pr.normal * pr.phi;
            }
        }
        var gone = false;
        for (var a = 0u; a < 3u; a++) {
            let lo = params.box_lo[a];
            let hi = params.box_hi[a];
            if params.periodic[a] != 0u {
                let span = hi - lo;
                let r = (x[a] - lo) - floor((x[a] - lo) / span) * span;
                x[a] = lo + select(r, 0.0, r >= span);
                continue;
            }
            for (var e = 0u; e < 2u; e++) {
                let side = 2u * a + e;
                var out = x[a] < lo;
                if e == 1u {
                    out = x[a] > hi;
                }
                if out {
                    if side_law(side) == 0u {
                        gone = true;
                    } else {
                        x[a] = clamp(x[a], lo, hi);
                    }
                }
            }
        }
        let cr = transpose(c);
        let t = up.tau;
        let bn = up.b;
        p.x = vec4f(x, temperature.x);
        p.v = vec4f(v, temperature.y);
        p.c0 = vec4f(cr[0], j_own);
        p.c1 = vec4f(cr[1], up.plastic);
        p.c2 = vec4f(cr[2], up.shear_rate);
        p.b = vec4f(bn[0][0], bn[1][1], bn[2][2], bn[1][0]);
        p.tau = vec4f(t[0][0], t[1][1], t[2][2], t[1][0]);
        p.bt = vec4f(bn[2][1], bn[2][0], t[2][1], t[2][0]);
        let speed = length(v);
        let gravity = bodies[body(p)].xyz;
        let potential = -mass * dot(gravity, x + params.origin.xyz);
        let kinetic = 0.5 * mass * speed * speed;
        let stored = up.stored * volume;
        sums[3] = up.dissipated * volume;
        if gone {
            p.info.z = 1u;
            sums[5] = mass;
            sums[6] = kinetic + potential + stored;
            sums[7] = 1.0;
        } else {
            sums[0] = kinetic;
            sums[1] = potential;
            sums[2] = stored;
            sums[4] = speed;
        }
        particles[pi] = p;
    }
    for (var k = 0u; k < SUMS; k++) {
        let sum = fold64(lid, sums[k], k == 4u);
        if lid == 0u {
            partials[params.offsets.y + g * SUMS + k] = sum;
        }
    }
}

// The sums of the step, the time, and whether the run waits for the processor: a frame is
// due, a history point, or nothing is left.
@compute @workgroup_size(64)
fn finish(@builtin(local_invocation_index) lid: u32) {
    var sums = array<f32, 8>(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for (var k = 0u; k < SUMS; k++) {
        var value = 0.0;
        for (var g = lid; g < params.particle_groups; g += 64u) {
            let v = partials[params.offsets.y + g * SUMS + k];
            if k == 4u {
                value = max(value, v);
            } else {
                value += v;
            }
        }
        sums[k] = fold64(lid, value, k == 4u);
    }
    let slots = params.slots;
    for (var i = lid; i < slots * LOAD; i += 64u) {
        loads[i] += partials[params.offsets.x + i];
    }
    if lid != 0u {
        return;
    }
    ctl.kinetic = sums[0];
    ctl.potential = sums[1];
    ctl.stored = sums[2];
    ctl.dissipated = add2(ctl.dissipated, sums[3]);
    ctl.max_speed = sums[4];
    ctl.top_speed = max(ctl.top_speed, sums[4]);
    ctl.out_mass = add2(ctl.out_mass, sums[5]);
    ctl.out_energy = add2(ctl.out_energy, sums[6]);
    ctl.gone += u32(sums[7] + 0.5);
    var work = 0.0;
    for (var slot = 0u; slot + 1u < slots; slot++) {
        work += partials[params.offsets.x + slot * LOAD + 6u];
    }
    ctl.work = add2(ctl.work, work);
    ctl.loss = add2(ctl.loss, partials[params.offsets.x + (slots - 1u) * LOAD]);
    ctl.samples += 1u;
    ctl.steps += 1u;
    ctl.tl += ctl.dt;
    if ctl.hit != 0u {
        ctl.reason |= FRAME;
        ctl.running = 0u;
    }
    if ctl.tl >= ctl.history {
        ctl.reason |= HISTORY;
        ctl.running = 0u;
    }
    if ctl.gone >= params.periodic.w {
        ctl.reason |= EMPTY;
        ctl.running = 0u;
    }
}

// Per workgroup of particles and per body: the particles left, the sum of their
// temperatures and the highest one, from the start of `partials` (between steps only). The
// particles of a body all have the same mass.
@compute @workgroup_size(64)
fn body_sums(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) groups: vec3u,
             @builtin(local_invocation_index) lid: u32) {
    let p = thread_index(wid, groups, lid, 64u);
    let g = group_index(wid, groups);
    var temperature = 0.0;
    var mine = NONE;
    if p < params.periodic.w {
        let part = particles[p];
        if part.info.z == 0u {
            mine = body(part);
            temperature = part.x.w + part.v.w;
        }
    }
    let count = arrayLength(&bodies);
    for (var b = 0u; b < count; b++) {
        let here = mine == b;
        let n = fold64(lid, select(0.0, 1.0, here), false);
        let sum = fold64(lid, select(0.0, temperature, here), false);
        let hottest = fold64(lid, select(-3.0e38, temperature, here), true);
        if lid == 0u {
            partials[(g * count + b) * 3u] = n;
            partials[(g * count + b) * 3u + 1u] = sum;
            partials[(g * count + b) * 3u + 2u] = hottest;
        }
    }
}
