// Partial sums of a dot product: workgroup `w` of `GROUPS` adds up the products of a
// strided part of the vectors and writes it to `partial[slot * GROUPS + w]`; the scalar
// kernel adds the partial sums.

const GROUPS: u32 = 256u;

struct Params {
    n: u32,
    slot: u32,
    pad: u32,
    pad2: u32,
}

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read> a: array<f64>;
@group(0) @binding(2) var<storage, read> b: array<f64>;
@group(0) @binding(3) var<storage, read_write> partial: array<f64>;

var<workgroup> sums: array<f64, 256>;

@compute @workgroup_size(256)
fn dot(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    var sum = f64(0.0);
    var i = wid.x * 256u + li;
    let stride = GROUPS * 256u;
    while i < params.n {
        sum += a[i] * b[i];
        i += stride;
    }
    sums[li] = sum;
    workgroupBarrier();
    var half = 128u;
    while half > 0u {
        if li < half {
            sums[li] += sums[li + half];
        }
        workgroupBarrier();
        half = half / 2u;
    }
    if li == 0u {
        partial[params.slot * GROUPS + wid.x] = sums[0];
    }
}
