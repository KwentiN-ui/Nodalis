// The internal forces of the displacement increments (`K Δu`) and the viscous forces of the
// velocities (`C v`) in single precision, both in one pass over the common pattern.
//
// Sliced storage: slice `s` holds `SLICE` rows (one subgroup of threads), entry `j` of all
// of them next to each other (`start[s] + j * SLICE + lane`), so that the threads of a
// subgroup read neighbouring values. `rows[s * SLICE + lane]` is the row of a position.
// Each entry holds the value of `K` and of `C`.

const SLICE: u32 = 32u;

struct Dims {
    // Rows.
    n: u32,
    // Workgroups along x, for dispatches folded into two dimensions.
    groups_x: u32,
    // Positions (slices times `SLICE`).
    positions: u32,
    pad: u32,
}

@group(0) @binding(0) var<uniform> dims: Dims;
// The displacement increment and the velocity of each unknown.
@group(0) @binding(1) var<storage, read> dv: array<vec2<f32>>;
@group(0) @binding(2) var<storage, read_write> fs: array<f32>;
@group(0) @binding(3) var<storage, read> start: array<u32>;
@group(0) @binding(4) var<storage, read> rows: array<u32>;
@group(0) @binding(5) var<storage, read> col: array<u32>;
@group(0) @binding(6) var<storage, read> val: array<vec2<f32>>;

@compute @workgroup_size(256)
fn product(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let t = (wid.y * dims.groups_x + wid.x) * 256u + li;
    if t >= dims.positions {
        return;
    }
    let row = rows[t];
    if row >= dims.n {
        return;
    }
    let slice = t / SLICE;
    let lane = t % SLICE;
    let first = start[slice];
    let width = (start[slice + 1u] - first) / SLICE;
    var s = vec2<f32>(0.0, 0.0);
    for (var j = 0u; j < width; j++) {
        let k = first + j * SLICE + lane;
        s += val[k] * dv[col[k]];
    }
    fs[row] = s.x;
    fs[dims.n + row] = s.y;
}
