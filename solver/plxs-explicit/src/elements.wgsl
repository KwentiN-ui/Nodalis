// The internal and viscous forces of one family of elements, one thread per element (see
// `elements.rs`): the displacements relative to the element's first node in double
// precision, then everything in single precision. Element data are stored value by value
// (`[k * count + e]`), so that neighbouring threads read neighbouring memory; the forces
// go to the element force buffers at `base + k * count + e`, which `gather.wgsl` sums per
// DOF.
//
// NN (nodes), NP (points), HG (hourglass control) and NL (large deformations) are set per
// family.

const NN: u32 = __NN__u;
const NP: u32 = __NP__u;
const HG: bool = __HG__;
const NL: bool = __NL__;
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
// The state variables of the plastic elements: `[(ip * STATE + s) * count + e]`.
@group(0) @binding(11) var<storage, read_write> history: array<f64>;

// Values per material block (see `gpu.rs`) and state variables per point.
const MAT: u32 = 44u;
const STATE: u32 = 14u;

const HOURGLASS = array<array<f32, 8>, 4>(
    array<f32, 8>(1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0),
    array<f32, 8>(1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0),
    array<f32, 8>(1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0),
    array<f32, 8>(-1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0),
);

// The natural coordinates of the brick's nodes.
const CORNERS = array<vec3<f32>, 8>(
    vec3<f32>(-1.0, -1.0, -1.0),
    vec3<f32>(1.0, -1.0, -1.0),
    vec3<f32>(1.0, 1.0, -1.0),
    vec3<f32>(-1.0, 1.0, -1.0),
    vec3<f32>(-1.0, -1.0, 1.0),
    vec3<f32>(1.0, -1.0, 1.0),
    vec3<f32>(1.0, 1.0, 1.0),
    vec3<f32>(-1.0, 1.0, 1.0),
);

// `det(m) m⁻ᵀ` of a matrix with the columns a, b, c.
fn cofactors(m: mat3x3<f32>) -> mat3x3<f32> {
    return mat3x3<f32>(cross(m[1], m[2]), cross(m[2], m[0]), cross(m[0], m[1]));
}

// Value and slope of the piecewise linear curve at `start` (`len` pairs), constant beyond
// its ends.
fn curve(start: u32, len: u32, x: f32) -> vec2<f32> {
    if len == 0u {
        return vec2<f32>(0.0, 0.0);
    }
    let x0 = materials[start];
    if x < x0 {
        return vec2<f32>(materials[start + 1u], 0.0);
    }
    for (var i = 0u; i + 1u < len; i++) {
        let a = start + 2u * i;
        let xb = materials[a + 2u];
        if x <= xb {
            let slope = (materials[a + 3u] - materials[a + 1u]) / (xb - materials[a]);
            return vec2<f32>(materials[a + 1u] + (x - materials[a]) * slope, slope);
        }
    }
    return vec2<f32>(materials[start + 2u * len - 1u], 0.0);
}

// The radial return of the processor's J2 law (rate independent) for the material block
// `m` at point `ip` of element `e`: the stress; the state variables advance.
fn j2(m: u32, strain: array<f32, 6>, ip: u32, c: u32, e: u32) -> array<f32, 6> {
    let mu = materials[m + 37u];
    let bulk = materials[m + 38u];
    let iso = vec2<u32>(u32(materials[m + 39u]), u32(materials[m + 40u]));
    let kin = vec2<u32>(u32(materials[m + 41u]), u32(materials[m + 42u]));
    let at = ip * STATE;
    let peeq = history[at * c + e];
    var el = array<f32, 6>();
    for (var k = 0u; k < 6u; k++) {
        var total = strain[k];
        if k >= 3u {
            total = 0.5 * total;
        }
        el[k] = f32(f64(total) - history[(at + 1u + k) * c + e]);
    }
    let trace = el[0] + el[1] + el[2];
    let mean = trace / 3.0;
    var s = array<f32, 6>();
    var xi = array<f32, 6>();
    for (var k = 0u; k < 6u; k++) {
        var d = el[k];
        if k < 3u {
            d -= mean;
        }
        s[k] = 2.0 * mu * d;
        xi[k] = s[k] - f32(history[(at + 7u + k) * c + e]);
    }
    let xn = sqrt(xi[0] * xi[0] + xi[1] * xi[1] + xi[2] * xi[2]
        + 2.0 * (xi[3] * xi[3] + xi[4] * xi[4] + xi[5] * xi[5]));
    let q = 1.22474487139 * xn;
    let p = f32(peeq);
    let yield_stress = curve(iso.x, iso.y, p).x;
    if q - yield_stress > 1e-9 * max(yield_stress, 0.0) && xn > 0.0 {
        let kappa0 = curve(kin.x, kin.y, p).x;
        var lo = 0.0;
        var hi = q / (3.0 * mu);
        var dg = (q - yield_stress)
            / (3.0 * mu + curve(kin.x, kin.y, p).y + curve(iso.x, iso.y, p).y);
        for (var it = 0u; it < 40u; it++) {
            let ki = curve(kin.x, kin.y, p + dg);
            let is = curve(iso.x, iso.y, p + dg);
            let g = q - 3.0 * mu * dg - (ki.x - kappa0) - is.x;
            if abs(g) <= 1e-6 * max(q, 1.0) {
                break;
            }
            if g > 0.0 {
                lo = dg;
            } else {
                hi = dg;
            }
            let next = dg + g / (3.0 * mu + ki.y + is.y);
            if next > lo && next < hi {
                dg = next;
            } else {
                dg = 0.5 * (lo + hi);
            }
        }
        dg = clamp(dg, 0.0, q / (3.0 * mu));
        let delta_kappa = curve(kin.x, kin.y, p + dg).x - kappa0;
        for (var k = 0u; k < 6u; k++) {
            let n = xi[k] / xn;
            let dep = dg * 1.22474487139 * n;
            history[(at + 1u + k) * c + e] += f64(dep);
            history[(at + 7u + k) * c + e] += f64(delta_kappa * 0.81649658093 * n);
            s[k] -= 2.0 * mu * dep;
        }
        history[at * c + e] = peeq + f64(dg);
    }
    for (var k = 0u; k < 3u; k++) {
        s[k] += bulk * trace;
    }
    return s;
}

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
    // The current volume and `V g` (fully integrated elements under NLGEOM).
    var gc: array<f32, ND>;
    var volume = 0.0;
    var reference = 0.0;
    let u0 = vec3<f64>(u[dofs[e]], u[dofs[c + e]], u[dofs[2u * c + e]]);
    for (var k = 0u; k < ND; k++) {
        let d = dofs[k * c + e];
        ue[k] = f32(u[d] - u0[k % 3u]);
        ve[k] = f32(v[d]);
        f[k] = 0.0;
        fv[k] = 0.0;
        gc[k] = 0.0;
    }
    let m = material[e] * MAT;
    let kind = u32(materials[m]);
    for (var ip = 0u; ip < NP; ip++) {
        let at = ip * PS;
        let w = gradients[at * c + e];
        reference += w;
        // Column j holds ∂u/∂X_j: h[j][k] = H_kj.
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
            let ua = vec3<f32>(ue[3u * a], ue[3u * a + 1u], ue[3u * a + 2u]);
            h += mat3x3<f32>(ga.x * ua, ga.y * ua, ga.z * ua);
        }
        // Engineering strain [xx, yy, zz, xy, yz, zx]: small, or Green-Lagrange.
        var strain = array<f32, 6>(
            h[0][0], h[1][1], h[2][2],
            h[1][0] + h[0][1], h[2][1] + h[1][2], h[0][2] + h[2][0],
        );
        if NL {
            strain[0] += 0.5 * dot(h[0], h[0]);
            strain[1] += 0.5 * dot(h[1], h[1]);
            strain[2] += 0.5 * dot(h[2], h[2]);
            strain[3] += dot(h[0], h[1]);
            strain[4] += dot(h[1], h[2]);
            strain[5] += dot(h[2], h[0]);
        }
        var s = array<f32, 6>();
        if kind == 1u {
            s = j2(m, strain, ip, c, e);
        } else {
            for (var r = 0u; r < 6u; r++) {
                var sum = 0.0;
                for (var q = 0u; q < 6u; q++) {
                    sum += materials[m + 1u + 6u * r + q] * strain[q];
                }
                s[r] = sum;
            }
        }
        var p = mat3x3<f32>(
            vec3<f32>(s[0], s[3], s[5]),
            vec3<f32>(s[3], s[1], s[4]),
            vec3<f32>(s[5], s[4], s[2]),
        );
        let deformation = mat3x3<f32>(
            vec3<f32>(1.0, 0.0, 0.0),
            vec3<f32>(0.0, 1.0, 0.0),
            vec3<f32>(0.0, 0.0, 1.0),
        ) + h;
        if NL {
            // P = F S.
            p = deformation * p;
        }
        for (var a = 0u; a < NN; a++) {
            let ga = vec3<f32>(g[3u * a], g[3u * a + 1u], g[3u * a + 2u]);
            let fa = w * (p * ga);
            f[3u * a] += fa.x;
            f[3u * a + 1u] += fa.y;
            f[3u * a + 2u] += fa.z;
        }
        if NL && !HG {
            // J F⁻ᵀ ∇N.
            let cof = cofactors(deformation);
            volume += w * dot(deformation[0], cof[0]);
            for (var a = 0u; a < NN; a++) {
                let ga = vec3<f32>(g[3u * a], g[3u * a + 1u], g[3u * a + 2u]);
                let x = w * (cof * ga);
                gc[3u * a] += x.x;
                gc[3u * a + 1u] += x.y;
                gc[3u * a + 2u] += x.z;
            }
        }
    }
    // The mean gradients of the viscosity: the reference ones, or the current ones.
    var mean: array<f32, ND>;
    if !NL {
        volume = reference;
        for (var k = 0u; k < ND; k++) {
            mean[k] = viscosity[(1u + k) * c + e];
        }
    } else if !HG {
        for (var k = 0u; k < ND; k++) {
            mean[k] = gc[k] / volume;
        }
    } else {
        // The brick at its current nodal positions, 2 × 2 × 2 points.
        let p = 0.57735026919;
        volume = 0.0;
        for (var k = 0u; k < ND; k++) {
            mean[k] = 0.0;
        }
        for (var i = 0u; i < 8u; i++) {
            let xi = p * CORNERS[i];
            var j = mat3x3<f32>();
            for (var a = 0u; a < 8u; a++) {
                let ca = CORNERS[a];
                let fa = vec3<f32>(1.0, 1.0, 1.0) + ca * xi;
                let dn = 0.125 * vec3<f32>(ca.x * fa.y * fa.z, ca.y * fa.x * fa.z, ca.z * fa.x * fa.y);
                let x = vec3<f32>(
                    control[(2u + 3u * a) * c + e] + ue[3u * a],
                    control[(3u + 3u * a) * c + e] + ue[3u * a + 1u],
                    control[(4u + 3u * a) * c + e] + ue[3u * a + 2u],
                );
                // Column m holds ∂x/∂ξ_m.
                j += mat3x3<f32>(dn.x * x, dn.y * x, dn.z * x);
            }
            let cof = cofactors(j);
            volume += dot(j[0], cof[0]);
            for (var a = 0u; a < 8u; a++) {
                let ca = CORNERS[a];
                let fa = vec3<f32>(1.0, 1.0, 1.0) + ca * xi;
                let dn = 0.125 * vec3<f32>(ca.x * fa.y * fa.z, ca.y * fa.x * fa.z, ca.z * fa.x * fa.y);
                let x = cof * dn;
                mean[3u * a] += x.x;
                mean[3u * a + 1u] += x.y;
                mean[3u * a + 2u] += x.z;
            }
        }
        for (var k = 0u; k < ND; k++) {
            mean[k] /= volume;
        }
    }
    // Bulk viscosity: `c V` of the reference over its volume, times the current volume.
    var rate = 0.0;
    for (var k = 0u; k < ND; k++) {
        rate += mean[k] * ve[k];
    }
    let q = viscosity[e] / reference * volume * rate;
    for (var k = 0u; k < ND; k++) {
        fv[k] += q * mean[k];
    }
    if HG {
        // One point: `g` holds its derivatives.
        let epsilon = control[e];
        let kappa = control[c + e];
        for (var i = 0u; i < 4u; i++) {
            var hx = vec3<f32>(0.0, 0.0, 0.0);
            var hm = vec3<f32>(0.0, 0.0, 0.0);
            for (var a = 0u; a < 8u; a++) {
                let x = vec3<f32>(
                    control[(2u + 3u * a) * c + e],
                    control[(3u + 3u * a) * c + e],
                    control[(4u + 3u * a) * c + e],
                );
                hx += HOURGLASS[i][a] * x;
                if NL {
                    hm += HOURGLASS[i][a] * (x + vec3<f32>(ue[3u * a], ue[3u * a + 1u], ue[3u * a + 2u]));
                }
            }
            if !NL {
                hm = hx;
            }
            var stiff = array<f32, 8>();
            var visc = array<f32, 8>();
            for (var a = 0u; a < 8u; a++) {
                stiff[a] = HOURGLASS[i][a] - hx.z * g[3u * a + 2u];
                visc[a] = HOURGLASS[i][a]
                    - dot(hm, vec3<f32>(mean[3u * a], mean[3u * a + 1u], mean[3u * a + 2u]));
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
