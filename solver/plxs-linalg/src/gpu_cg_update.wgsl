// Vector steps of the conjugate gradients. op 0: x += alpha p, r -= alpha q;
// op 1: p = z + beta p; op 2: r = b, x = 0; op 3: p = z.

struct Params {
    n: u32,
    op: u32,
    groups_x: u32,
    pad: u32,
}

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read> s: array<f64>;
@group(0) @binding(2) var<storage, read> q: array<f64>;
@group(0) @binding(3) var<storage, read> z: array<f64>;
@group(0) @binding(4) var<storage, read_write> x: array<f64>;
@group(0) @binding(5) var<storage, read_write> r: array<f64>;
@group(0) @binding(6) var<storage, read_write> p: array<f64>;

@compute @workgroup_size(256)
fn update(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let i = (wid.y * params.groups_x + wid.x) * 256u + li;
    if i >= params.n {
        return;
    }
    switch params.op {
        case 0u: {
            let alpha = s[1];
            x[i] += alpha * p[i];
            r[i] -= alpha * q[i];
        }
        case 1u: {
            p[i] = z[i] + s[2] * p[i];
        }
        case 2u: {
            // q holds b here.
            r[i] = q[i];
            x[i] = f64(0.0);
        }
        default: {
            p[i] = z[i];
        }
    }
}
