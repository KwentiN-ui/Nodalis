// The internal and viscous forces per DOF: the sum of the element forces at the positions
// listed for the DOF (`ptr[i]..ptr[i + 1]` in `positions`), in double precision.

struct Dims {
    n: u32,
    groups_x: u32,
    positions: u32,
    relative: u32,
}

@group(0) @binding(0) var<uniform> dims: Dims;
@group(0) @binding(1) var<storage, read> ptr: array<u32>;
@group(0) @binding(2) var<storage, read> positions: array<u32>;
@group(0) @binding(3) var<storage, read> fint: array<f32>;
@group(0) @binding(4) var<storage, read> fvis: array<f32>;
@group(0) @binding(5) var<storage, read_write> fs: array<f64>;

@compute @workgroup_size(256)
fn gather(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let i = (wid.y * dims.groups_x + wid.x) * 256u + li;
    if i >= dims.n {
        return;
    }
    var si = f64(0.0);
    var sv = f64(0.0);
    for (var k = ptr[i]; k < ptr[i + 1u]; k++) {
        let p = positions[k];
        si += f64(fint[p]);
        sv += f64(fvis[p]);
    }
    fs[i] = si;
    fs[dims.n + i] = sv;
}
