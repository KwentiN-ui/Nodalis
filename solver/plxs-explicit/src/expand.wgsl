// The displacements and velocities of the DOFs from the reduced unknowns: `u = T q + g`,
// `v = T q̇`, and the increment of the displacements for the work of the step. `T` by rows:
// `index` holds the row pointers (n + 1), then the columns.

struct Dims {
    n: u32,
    groups_x: u32,
    base: u32,
    pad: u32,
}

@group(0) @binding(0) var<uniform> dims: Dims;
@group(0) @binding(1) var<storage, read> q: array<f64>;
@group(0) @binding(2) var<storage, read> qd: array<f64>;
@group(0) @binding(3) var<storage, read> g: array<f64>;
@group(0) @binding(4) var<storage, read> index: array<u32>;
@group(0) @binding(5) var<storage, read> values: array<f64>;
@group(0) @binding(6) var<storage, read_write> u: array<f64>;
@group(0) @binding(7) var<storage, read_write> v: array<f64>;
@group(0) @binding(8) var<storage, read_write> du: array<f64>;

@compute @workgroup_size(256)
fn expand(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let i = (wid.y * dims.groups_x + wid.x) * 256u + li;
    if i >= dims.n {
        return;
    }
    let columns = dims.n + 1u;
    var x = g[i];
    var y = f64(0.0);
    for (var k = index[i]; k < index[i + 1u]; k++) {
        let j = index[columns + k];
        x += values[k] * q[j];
        y += values[k] * qd[j];
    }
    du[i] = x - u[i];
    u[i] = x;
    v[i] = y;
}
