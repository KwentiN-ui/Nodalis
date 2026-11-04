// The scalar steps of the conjugate gradients, one thread. `s` holds
// 0 rz, 1 alpha, 2 beta, 3 b·b, 4 r·r, 5 done (1 converged, 2 not positive definite),
// 6 iterations, 7 tolerance² · b·b (tolerance² before the start), 8 maximum iterations.
// done 3: the maximum number of iterations reached.
// Partial sums: slot 0 p·q, 1 r·r, 2 r·z, 3 z·q, 4 b·b.

const GROUPS: u32 = 256u;

struct Params {
    phase: u32,
    pad: u32,
    pad2: u32,
    pad3: u32,
}

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read> partial: array<f64>;
@group(0) @binding(2) var<storage, read_write> s: array<f64>;

fn total(slot: u32) -> f64 {
    var sum = f64(0.0);
    for (var w = 0u; w < GROUPS; w++) {
        sum += partial[slot * GROUPS + w];
    }
    return sum;
}

@compute @workgroup_size(1)
fn scalar() {
    let zero = f64(0.0);
    let one = f64(1.0);
    switch params.phase {
        // Start: rz = r·z, b·b.
        case 0u: {
            s[0] = total(2u);
            s[3] = total(4u);
            s[7] = s[7] * s[3];
            s[1] = zero;
            s[2] = zero;
            s[5] = zero;
            s[6] = zero;
            if s[3] == zero {
                s[5] = one;
            }
        }
        // alpha = rz / p·q.
        case 1u: {
            if s[5] != zero {
                s[1] = zero;
            } else {
                let pq = total(0u);
                // Also NaN: a comparison with NaN is false.
                if !(pq > zero) || !(pq < f64(1e300)) {
                    s[5] = f64(2.0);
                    s[1] = zero;
                } else {
                    s[1] = s[0] / pq;
                }
            }
        }
        // The residual after the update.
        case 2u: {
            if s[5] == zero {
                s[6] += one;
                let rr = total(1u);
                s[4] = rr;
                if rr <= s[7] {
                    s[5] = one;
                } else if s[6] >= s[8] {
                    s[5] = f64(3.0);
                }
            }
        }
        // beta (Polak-Ribière), rz.
        default: {
            if s[5] != zero {
                s[2] = zero;
            } else {
                let rz_new = total(2u);
                s[2] = -s[1] * total(3u) / s[0];
                s[0] = rz_new;
            }
        }
    }
}
