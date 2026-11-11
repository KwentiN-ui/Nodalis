// The forces of the constant stiffness blocks, one thread per block row: `K u` in double
// precision (the blocks act on any DOFs, rotations too, so no displacements relative to a
// node keep single precision for rigid motions), written to the element force buffer at
// `base + row`, which `gather.wgsl` sums per DOF.

struct Dims {
    count: u32,
    groups_x: u32,
    base: u32,
    pad: u32,
}

@group(0) @binding(0) var<uniform> dims: Dims;
@group(0) @binding(1) var<storage, read> u: array<f64>;
// Row `r` holds the entries `ptr[r]..ptr[r + 1]` of `columns` (DOFs) and `values`.
@group(0) @binding(2) var<storage, read> ptr: array<u32>;
@group(0) @binding(3) var<storage, read> columns: array<u32>;
@group(0) @binding(4) var<storage, read> values: array<f64>;
@group(0) @binding(5) var<storage, read_write> fint: array<f32>;

@compute @workgroup_size(64)
fn blocks(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let row = (wid.y * dims.groups_x + wid.x) * 64u + li;
    if row >= dims.count {
        return;
    }
    var sum = f64(0.0);
    for (var k = ptr[row]; k < ptr[row + 1u]; k++) {
        sum += values[k] * u[columns[k]];
    }
    fint[dims.base + row] = f32(sum);
}
