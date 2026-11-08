// The work of the prescribed displacements: the reactions `M (T a + g̈) + f_int + f_v − p`
// (`M T a − residual`) on the DOFs that move by `g`, by the trapezoidal rule over the
// step. `T` by rows: `index` holds the row pointers (n + 1), then the columns.

struct Dims {
    n: u32,
    groups_x: u32,
    base: u32,
    pad: u32,
}

@group(0) @binding(0) var<uniform> dims: Dims;
@group(0) @binding(1) var<storage, read> a: array<f64>;
@group(0) @binding(2) var<storage, read> index: array<u32>;
@group(0) @binding(3) var<storage, read> values: array<f64>;
@group(0) @binding(4) var<storage, read> mass: array<f64>;
@group(0) @binding(5) var<storage, read> residual: array<f64>;
// The increment of the prescribed displacements at `4n`.
@group(0) @binding(6) var<storage, read> g: array<f64>;
// The reactions of the step before, then the work so far.
@group(0) @binding(7) var<storage, read_write> reaction: array<f64>;

@compute @workgroup_size(256)
fn react(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let i = (wid.y * dims.groups_x + wid.x) * 256u + li;
    let n = dims.n;
    if i >= n {
        return;
    }
    let columns = n + 1u;
    var ta = f64(0.0);
    for (var k = index[i]; k < index[i + 1u]; k++) {
        ta += values[k] * a[index[columns + k]];
    }
    let r = mass[i] * ta - residual[i];
    let dg = g[4u * n + i];
    if dg != f64(0.0) {
        reaction[n + i] += dg * f64(0.5) * (reaction[i] + r);
    }
    reaction[i] = r;
}
