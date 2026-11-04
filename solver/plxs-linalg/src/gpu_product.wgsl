// Block products of the multigrid V-cycle on the GPU (single precision), see gpu.rs.

// Block dimensions of the matrix of a product, fixed per pipeline so the inner loop
// unrolls.
override BR: u32 = 3u;
override BC: u32 = 3u;
// Block rows per slice of the sliced storage (gpu.rs, SLICE).
const SLICE: u32 = 32u;

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

// Sliced block rows: slice `s` holds the blocks of `SLICE` block rows, entry `j` of all
// of them next to each other (`start[s] + j * SLICE + b`), so that neighbouring threads
// read neighbouring values. `rows[s * SLICE + b]` is the block row of position `b`.
@group(0) @binding(1) var<storage, read> start: array<u32>;
@group(0) @binding(2) var<storage, read> rows: array<u32>;
@group(0) @binding(3) var<storage, read> col: array<u32>;
@group(0) @binding(4) var<storage, read> val: array<f32>;
@group(0) @binding(5) var<storage, read> x: array<f32>;
@group(0) @binding(6) var<storage, read> y0: array<f32>;
@group(0) @binding(7) var<storage, read_write> y: array<f32>;

// One thread per scalar row.
@compute @workgroup_size(64)
fn product(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let t = (wid.y * params.groups_x + wid.x) * 64u + li;
    let per_slice = SLICE * BR;
    let slice = t / per_slice;
    let local = t % per_slice;
    let b = local / BR;
    let r = local % BR;
    let position = slice * SLICE + b;
    if position >= params.n {
        return;
    }
    let first = start[slice];
    let width = (start[slice + 1u] - first) / SLICE;
    var sum = 0.0;
    for (var j = 0u; j < width; j++) {
        let k = first + j * SLICE + b;
        let c = col[k] * BC;
        let base = k * (BR * BC) + r * BC;
        for (var q = 0u; q < BC; q++) {
            sum += val[base + q] * x[c + q];
        }
    }
    let row = rows[position] * BR + r;
    if params.op != 0u {
        y[row] = y0[row] - sum;
    } else {
        y[row] = sum;
    }
}

