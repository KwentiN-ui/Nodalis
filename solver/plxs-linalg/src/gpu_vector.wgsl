// Vector operations of the multigrid V-cycle on the GPU (single precision), see gpu.rs.

struct Params {
    // Block rows (products) or entries (vector operations).
    n: u32,
    // Products: 1 for `y = y0 - A x`, 0 for `y = A x`. Vector operations: the operation.
    op: u32,
    // Workgroups along x, for dispatches folded into two dimensions.
    groups_x: u32,
    pad: u32,
    c1: f32,
    c2: f32,
    pad2: f32,
    pad3: f32,
}

@group(0) @binding(0) var<uniform> params: Params;

@group(0) @binding(1) var<storage, read> a: array<f32>;
@group(0) @binding(2) var<storage, read> diag: array<f32>;
@group(0) @binding(3) var<storage, read_write> v: array<f32>;

// 0: v = a; 1: v += a; 2: v -= a; 3: v = c1 diag a; 4: v = c1 v + c2 diag a.
@compute @workgroup_size(256)
fn vector(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let i = (wid.y * params.groups_x + wid.x) * 256u + li;
    if i >= params.n {
        return;
    }
    switch params.op {
        case 0u: { v[i] = a[i]; }
        case 1u: { v[i] += a[i]; }
        case 2u: { v[i] -= a[i]; }
        case 3u: { v[i] = params.c1 * diag[i] * a[i]; }
        default: { v[i] = params.c1 * v[i] + params.c2 * diag[i] * a[i]; }
    }
}
