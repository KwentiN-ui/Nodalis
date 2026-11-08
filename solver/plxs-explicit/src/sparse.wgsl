// `y = A x` in double precision, one thread per row: the reduction `Tᵀ r` of the residual
// and the reduced accelerations `M_q⁻¹ r_q` of the clusters of the mass. `index` holds the
// row pointers (n + 1), then the columns.

struct Dims {
    n: u32,
    groups_x: u32,
    base: u32,
    pad: u32,
}

@group(0) @binding(0) var<uniform> dims: Dims;
@group(0) @binding(1) var<storage, read> index: array<u32>;
@group(0) @binding(2) var<storage, read> values: array<f64>;
@group(0) @binding(3) var<storage, read> x: array<f64>;
@group(0) @binding(4) var<storage, read_write> y: array<f64>;

@compute @workgroup_size(256)
fn sparse(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let i = (wid.y * dims.groups_x + wid.x) * 256u + li;
    if i >= dims.n {
        return;
    }
    let columns = dims.n + 1u;
    var s = f64(0.0);
    for (var k = index[i]; k < index[i + 1u]; k++) {
        s += values[k] * x[index[columns + k]];
    }
    y[i] = s;
}
