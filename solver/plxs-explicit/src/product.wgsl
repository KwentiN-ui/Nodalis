// The internal forces of the displacement increments (`K Δu`) and the viscous forces of the
// velocities (`C v`) in single precision, one row per thread.

struct Dims {
    n: u32,
    groups_x: u32,
    pad0: u32,
    pad1: u32,
}

@group(0) @binding(0) var<uniform> dims: Dims;
@group(0) @binding(1) var<storage, read> dv: array<f32>;
@group(0) @binding(2) var<storage, read_write> fs: array<f32>;
@group(0) @binding(3) var<storage, read> ptr: array<u32>;
@group(0) @binding(4) var<storage, read> col: array<u32>;
@group(0) @binding(5) var<storage, read> val: array<f32>;

fn row(wid: vec3<u32>, li: u32) -> u32 {
    return (wid.y * dims.groups_x + wid.x) * 256u + li;
}

@compute @workgroup_size(256)
fn stiffness(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let i = row(wid, li);
    if i >= dims.n {
        return;
    }
    var s = 0.0;
    for (var k = ptr[i]; k < ptr[i + 1u]; k++) {
        s += val[k] * dv[col[k]];
    }
    fs[i] = s;
}

@compute @workgroup_size(256)
fn viscous(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let i = row(wid, li);
    if i >= dims.n {
        return;
    }
    var s = 0.0;
    for (var k = ptr[i]; k < ptr[i + 1u]; k++) {
        s += val[k] * dv[dims.n + col[k]];
    }
    fs[dims.n + i] = s;
}
