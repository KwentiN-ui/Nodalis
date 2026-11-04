// The product `y = A x` of the conjugate gradients in double precision on the sliced
// storage of the finest operator (see gpu_product.wgsl for the layout).

override BR: u32 = 3u;
override BC: u32 = 3u;
const SLICE: u32 = 32u;

struct Params {
    n: u32,
    op: u32,
    groups_x: u32,
    pad: u32,
}

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read> start: array<u32>;
@group(0) @binding(2) var<storage, read> rows: array<u32>;
@group(0) @binding(3) var<storage, read> col: array<u32>;
@group(0) @binding(4) var<storage, read> val: array<f64>;
@group(0) @binding(5) var<storage, read> x: array<f64>;
@group(0) @binding(6) var<storage, read_write> y: array<f64>;

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
    var sum = f64(0.0);
    for (var j = 0u; j < width; j++) {
        let k = first + j * SLICE + b;
        let c = col[k] * BC;
        let base = k * (BR * BC) + r * BC;
        for (var q = 0u; q < BC; q++) {
            sum += val[base + q] * x[c + q];
        }
    }
    y[rows[position] * BR + r] = sum;
}
