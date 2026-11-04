// Between the double precision vectors of the conjugate gradients and the single
// precision ones of the V-cycle. op 0: lo = f32(hi); op 1: hi = f64(lo).

struct Params {
    n: u32,
    op: u32,
    groups_x: u32,
    pad: u32,
}

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read_write> hi: array<f64>;
@group(0) @binding(2) var<storage, read_write> lo: array<f32>;

@compute @workgroup_size(256)
fn convert(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let i = (wid.y * params.groups_x + wid.x) * 256u + li;
    if i >= params.n {
        return;
    }
    if params.op == 0u {
        lo[i] = f32(hi[i]);
    } else {
        hi[i] = f64(lo[i]);
    }
}
