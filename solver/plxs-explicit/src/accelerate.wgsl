// The forces at the new time in double precision (the reference forces plus the single
// precision products, the loads interpolated over the chunk, the mass-proportional
// damping), the work of the step per DOF (trapezoidal rule) and the accelerations.

struct Dims {
    n: u32,
    groups_x: u32,
    pad0: u32,
    pad1: u32,
}

struct Step {
    h: f64,
    c: f64,
    s: f64,
    alpha: f64,
}

@group(0) @binding(0) var<uniform> dims: Dims;
@group(0) @binding(1) var<uniform> step: Step;
@group(0) @binding(2) var<storage, read> v: array<f64>;
@group(0) @binding(3) var<storage, read_write> a: array<f64>;
@group(0) @binding(4) var<storage, read> du: array<f64>;
@group(0) @binding(5) var<storage, read> im: array<f64>;
// p at the start of the chunk, p at its end, the internal forces of the reference state.
@group(0) @binding(6) var<storage, read> loads: array<f64>;
@group(0) @binding(7) var<storage, read> fs: array<f32>;
// The internal, external and viscous forces of the step before, then the work of the
// internal, external and viscous forces so far.
@group(0) @binding(8) var<storage, read_write> energy: array<f64>;

@compute @workgroup_size(256)
fn accelerate(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let i = (wid.y * dims.groups_x + wid.x) * 256u + li;
    let n = dims.n;
    if i >= n {
        return;
    }
    let p = loads[i] + step.s * (loads[n + i] - loads[i]);
    let f_int = loads[2u * n + i] + f64(fs[i]);
    var m = f64(0.0);
    if im[i] > f64(0.0) {
        m = f64(1.0) / im[i];
    }
    let f_v = f64(fs[n + i]) + step.alpha * m * v[i];
    let d = du[i];
    let half = f64(0.5);
    energy[3u * n + i] += d * half * (energy[i] + f_int);
    energy[4u * n + i] += d * half * (energy[n + i] + p);
    energy[5u * n + i] += d * half * (energy[2u * n + i] + f_v);
    energy[i] = f_int;
    energy[n + i] = p;
    energy[2u * n + i] = f_v;
    a[i] = (p - f_int - f_v) * im[i];
}
