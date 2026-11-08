// The reduced velocities and displacements of one explicit time step:
// `q̇ += c a`, `q += h q̇`.

struct Dims {
    n: u32,
    groups_x: u32,
    base: u32,
    pad: u32,
}

struct Step {
    h: f64,
    c: f64,
    s: f64,
    alpha: f64,
    // `s` of the step before (the prescribed displacements' start of this step).
    s_before: f64,
    pad: f64,
}

@group(0) @binding(0) var<uniform> dims: Dims;
@group(0) @binding(1) var<uniform> step: Step;
@group(0) @binding(2) var<storage, read_write> q: array<f64>;
@group(0) @binding(3) var<storage, read_write> qd: array<f64>;
@group(0) @binding(4) var<storage, read> a: array<f64>;

@compute @workgroup_size(256)
fn advance(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let j = (wid.y * dims.groups_x + wid.x) * 256u + li;
    if j >= dims.n {
        return;
    }
    qd[j] += step.c * a[j];
    q[j] += step.h * qd[j];
}
