// The internal and viscous forces of one family of elements, one thread per element (see
// `elements.rs`): the displacements relative to the element's first node in double
// precision, then everything in single precision. Element data are stored value by value
// (`[k * count + e]`), so that neighbouring threads read neighbouring memory; the forces
// go to the element force buffers at `base + k * count + e`, which `gather.wgsl` sums per
// DOF.
//
// NN (nodes), NP (points) and HG (hourglass control) are set per family.

const NN: u32 = __NN__u;
const NP: u32 = __NP__u;
const HG: bool = __HG__;
const ND: u32 = 3u * NN;
const PS: u32 = 1u + ND;

struct Dims {
    count: u32,
    groups_x: u32,
    base: u32,
    pad: u32,
}

@group(0) @binding(0) var<uniform> dims: Dims;
@group(0) @binding(1) var<storage, read> u: array<f64>;
@group(0) @binding(2) var<storage, read> v: array<f64>;
@group(0) @binding(3) var<storage, read> dofs: array<u32>;
@group(0) @binding(4) var<storage, read> gradients: array<f32>;
@group(0) @binding(5) var<storage, read> viscosity: array<f32>;
@group(0) @binding(6) var<storage, read> control: array<f32>;
@group(0) @binding(7) var<storage, read> material: array<u32>;
@group(0) @binding(8) var<storage, read> materials: array<f32>;
@group(0) @binding(9) var<storage, read_write> fint: array<f32>;
@group(0) @binding(10) var<storage, read_write> fvis: array<f32>;

const HOURGLASS = array<array<f32, 8>, 4>(
    array<f32, 8>(1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0),
    array<f32, 8>(1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0),
    array<f32, 8>(1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0),
    array<f32, 8>(-1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0),
);

@compute @workgroup_size(64)
fn elements(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let e = (wid.y * dims.groups_x + wid.x) * 64u + li;
    let c = dims.count;
    if e >= c {
        return;
    }
    var ue: array<f32, ND>;
    var ve: array<f32, ND>;
    var f: array<f32, ND>;
    var fv: array<f32, ND>;
    var g: array<f32, ND>;
    let u0 = vec3<f64>(u[dofs[e]], u[dofs[c + e]], u[dofs[2u * c + e]]);
    for (var k = 0u; k < ND; k++) {
        let d = dofs[k * c + e];
        ue[k] = f32(u[d] - u0[k % 3u]);
        ve[k] = f32(v[d]);
        f[k] = 0.0;
        fv[k] = 0.0;
    }
    let m = material[e] * 36u;
    for (var ip = 0u; ip < NP; ip++) {
        let at = ip * PS;
        let w = gradients[at * c + e];
        var h = mat3x3<f32>();
        for (var a = 0u; a < NN; a++) {
            let ga = vec3<f32>(
                gradients[(at + 1u + 3u * a) * c + e],
                gradients[(at + 2u + 3u * a) * c + e],
                gradients[(at + 3u + 3u * a) * c + e],
            );
            g[3u * a] = ga.x;
            g[3u * a + 1u] = ga.y;
            g[3u * a + 2u] = ga.z;
            // h[c][k] = ∂u_k/∂X_c (column c of the matrix is the derivative along c).
            h += mat3x3<f32>(ga.x * vec3<f32>(ue[3u * a], ue[3u * a + 1u], ue[3u * a + 2u]),
                             ga.y * vec3<f32>(ue[3u * a], ue[3u * a + 1u], ue[3u * a + 2u]),
                             ga.z * vec3<f32>(ue[3u * a], ue[3u * a + 1u], ue[3u * a + 2u]));
        }
        // Engineering strain [xx, yy, zz, xy, yz, zx]; h[c][k] = H_kc.
        var strain = array<f32, 6>(
            h[0][0], h[1][1], h[2][2],
            h[1][0] + h[0][1], h[2][1] + h[1][2], h[0][2] + h[2][0],
        );
        var s = array<f32, 6>();
        for (var r = 0u; r < 6u; r++) {
            var sum = 0.0;
            for (var q = 0u; q < 6u; q++) {
                sum += materials[m + 6u * r + q] * strain[q];
            }
            s[r] = sum;
        }
        let sigma = mat3x3<f32>(
            vec3<f32>(s[0], s[3], s[5]),
            vec3<f32>(s[3], s[1], s[4]),
            vec3<f32>(s[5], s[4], s[2]),
        );
        for (var a = 0u; a < NN; a++) {
            let fa = w * (sigma * vec3<f32>(g[3u * a], g[3u * a + 1u], g[3u * a + 2u]));
            f[3u * a] += fa.x;
            f[3u * a + 1u] += fa.y;
            f[3u * a + 2u] += fa.z;
        }
    }
    // Bulk viscosity.
    var rate = 0.0;
    for (var k = 0u; k < ND; k++) {
        rate += viscosity[(1u + k) * c + e] * ve[k];
    }
    let q = viscosity[e] * rate;
    for (var k = 0u; k < ND; k++) {
        fv[k] += q * viscosity[(1u + k) * c + e];
    }
    if HG {
        // One point: `g` holds its derivatives.
        let epsilon = control[e];
        let kappa = control[c + e];
        for (var i = 0u; i < 4u; i++) {
            var hx = vec3<f32>(0.0, 0.0, 0.0);
            for (var a = 0u; a < 8u; a++) {
                hx += HOURGLASS[i][a] * vec3<f32>(
                    control[(2u + 3u * a) * c + e],
                    control[(3u + 3u * a) * c + e],
                    control[(4u + 3u * a) * c + e],
                );
            }
            var stiff = array<f32, 8>();
            var visc = array<f32, 8>();
            for (var a = 0u; a < 8u; a++) {
                stiff[a] = HOURGLASS[i][a] - hx.z * g[3u * a + 2u];
                visc[a] = HOURGLASS[i][a] - (hx.x * viscosity[(1u + 3u * a) * c + e]
                    + hx.y * viscosity[(2u + 3u * a) * c + e]
                    + hx.z * viscosity[(3u + 3u * a) * c + e]);
            }
            for (var k = 0u; k < 3u; k++) {
                var qs = 0.0;
                var qv = 0.0;
                for (var a = 0u; a < 8u; a++) {
                    qs += stiff[a] * ue[3u * a + k];
                    qv += visc[a] * ve[3u * a + k];
                }
                for (var a = 0u; a < 8u; a++) {
                    f[3u * a + k] += epsilon * stiff[a] * qs;
                    fv[3u * a + k] += kappa * visc[a] * qv;
                }
            }
        }
    }
    for (var k = 0u; k < ND; k++) {
        fint[dims.base + k * c + e] = f[k];
        fvis[dims.base + k * c + e] = fv[k];
    }
}
