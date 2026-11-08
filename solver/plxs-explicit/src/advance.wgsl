// Velocities and displacements of one explicit time step, and the single precision
// increments of the displacements (since the reference state) and the velocities that the
// products take.

struct Dims {
    n: u32,
    groups_x: u32,
    positions: u32,
    pad: u32,
}

struct Step {
    h: f64,
    c: f64,
    s: f64,
    alpha: f64,
}

@group(0) @binding(0) var<uniform> dims: Dims;
@group(0) @binding(1) var<uniform> step: Step;
@group(0) @binding(2) var<storage, read_write> u: array<f64>;
@group(0) @binding(3) var<storage, read_write> v: array<f64>;
@group(0) @binding(4) var<storage, read> a: array<f64>;
@group(0) @binding(5) var<storage, read_write> du: array<f64>;
@group(0) @binding(6) var<storage, read> uref: array<f64>;
@group(0) @binding(7) var<storage, read_write> dv: array<vec2<f32>>;
@group(0) @binding(8) var<storage, read> im: array<f64>;

@compute @workgroup_size(256)
fn advance(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let i = (wid.y * dims.groups_x + wid.x) * 256u + li;
    if i >= dims.n {
        return;
    }
    if im[i] > f64(0.0) {
        v[i] += step.c * a[i];
        let d = step.h * v[i];
        u[i] += d;
        du[i] = d;
    } else {
        du[i] = f64(0.0);
    }
    dv[i] = vec2<f32>(f32(u[i] - uref[i]), f32(v[i]));
}
