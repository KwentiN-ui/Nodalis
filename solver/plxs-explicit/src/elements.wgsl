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
// `m` at point `ip` of element `e` on the elastic trial strain `el` (tensor components):
// the stress and, where the point flows, the plastic strain increment. The equivalent
// plastic strain and the back stress advance; the plastic strain slots are the caller's.
struct Return {
    s: array<f32, 6>,
    dep: array<f32, 6>,
    flowed: bool,
}

fn radial(m: u32, el: array<f32, 6>, ip: u32, c: u32, e: u32) -> Return {
    let mu = materials[m + 37u];
    let bulk = materials[m + 38u];
    let iso = vec2<u32>(u32(materials[m + 39u]), u32(materials[m + 40u]));
    let kin = vec2<u32>(u32(materials[m + 41u]), u32(materials[m + 42u]));
    let at = ip * STATE;
    let peeq = history[at * c + e];
    var r = Return();
    let trace = el[0] + el[1] + el[2];
    let mean = trace / 3.0;
    var xi = array<f32, 6>();
    for (var k = 0u; k < 6u; k++) {
        var d = el[k];
        if k < 3u {
            d -= mean;
        }
        r.s[k] = 2.0 * mu * d;
        xi[k] = r.s[k] - f32(history[(at + 7u + k) * c + e]);
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
            r.dep[k] = dg * 1.22474487139 * n;
            history[(at + 7u + k) * c + e] += f64(delta_kappa * 0.81649658093 * n);
            r.s[k] -= 2.0 * mu * r.dep[k];
        }
        history[at * c + e] = peeq + f64(dg);
        r.flowed = true;
    }
    for (var k = 0u; k < 3u; k++) {
        r.s[k] += bulk * trace;
    }
    return r;
}

// The small-strain J2 law of the engineering strain: the stress.
fn j2(m: u32, strain: array<f32, 6>, ip: u32, c: u32, e: u32) -> array<f32, 6> {
    let at = ip * STATE;
    var el = array<f32, 6>();
    for (var k = 0u; k < 6u; k++) {
        var total = strain[k];
        if k >= 3u {
            total = 0.5 * total;
        }
        el[k] = f32(f64(total) - history[(at + 1u + k) * c + e]);
    }
    let r = radial(m, el, ip, c, e);
    if r.flowed {
        for (var k = 0u; k < 6u; k++) {
            history[(at + 1u + k) * c + e] += f64(r.dep[k]);
        }
    }
    return r.s;
}

// Eigenvalues and eigenvectors (`vectors[i][a]`: component i of the a-th) of a symmetric
// matrix by cyclic Jacobi rotations. Matrices here are indexed `m[i][j]` for row i and
// column j; all of them are symmetric.
struct Eigen {
    values: vec3<f32>,
    vectors: mat3x3<f32>,
}

fn eigen(m: mat3x3<f32>) -> Eigen {
    var a = m;
    var v = mat3x3<f32>(
        vec3<f32>(1.0, 0.0, 0.0),
        vec3<f32>(0.0, 1.0, 0.0),
        vec3<f32>(0.0, 0.0, 1.0),
    );
    for (var sweep = 0u; sweep < 10u; sweep++) {
        let off = abs(a[0][1]) + abs(a[0][2]) + abs(a[1][2]);
        let scale = abs(a[0][0]) + abs(a[1][1]) + abs(a[2][2]) + off;
        if off <= 1e-8 * scale {
            break;
        }
        for (var r = 0u; r < 3u; r++) {
            // The pairs (0, 1), (0, 2), (1, 2).
            let p = select(0u, 1u, r == 2u);
            let q = min(r + 1u, 2u);
            let apq = a[p][q];
            if apq == 0.0 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * apq);
            var t = 1.0;
            if theta != 0.0 {
                t = sign(theta) / (abs(theta) + sqrt(theta * theta + 1.0));
            }
            let cs = 1.0 / sqrt(t * t + 1.0);
            let sn = t * cs;
            for (var k = 0u; k < 3u; k++) {
                let akp = a[k][p];
                let akq = a[k][q];
                a[k][p] = cs * akp - sn * akq;
                a[k][q] = sn * akp + cs * akq;
            }
            for (var k = 0u; k < 3u; k++) {
                let apk = a[p][k];
                let aqk = a[q][k];
                a[p][k] = cs * apk - sn * aqk;
                a[q][k] = sn * apk + cs * aqk;
            }
            for (var k = 0u; k < 3u; k++) {
                let vkp = v[k][p];
                let vkq = v[k][q];
                v[k][p] = cs * vkp - sn * vkq;
                v[k][q] = sn * vkp + cs * vkq;
            }
        }
    }
    return Eigen(vec3<f32>(a[0][0], a[1][1], a[2][2]), v);
}

// `Σ_a f_a v_a v_aᵀ` of the eigenvectors `v`.
fn rebuild(v: mat3x3<f32>, f: vec3<f32>) -> mat3x3<f32> {
    var out = mat3x3<f32>();
    for (var i = 0u; i < 3u; i++) {
        for (var j = 0u; j < 3u; j++) {
            out[i][j] = f.x * v[i][0] * v[j][0] + f.y * v[i][1] * v[j][1] + f.z * v[i][2] * v[j][2];
        }
    }
    return out;
}

// `ln(1 + x)`, accurate near zero (2 artanh(x / (2 + x)) as a series).
fn log1p(x: f32) -> f32 {
    if abs(x) > 0.5 {
        return log(max(1.0 + x, 1e-30));
    }
    let t = x / (2.0 + x);
    let t2 = t * t;
    var sum = 0.0;
    var power = t;
    for (var k = 1u; k < 18u; k += 2u) {
        sum += power / f32(k);
        power *= t2;
    }
    return 2.0 * sum;
}

// `exp(x) − 1`, accurate near zero.
fn expm1(x: f32) -> f32 {
    if abs(x) > 0.5 {
        return exp(x) - 1.0;
    }
    var sum = 0.0;
    var term = x;
    for (var k = 2u; k < 14u; k++) {
        sum += term;
        term *= x / f32(k);
    }
    return sum;
}

// `(1 + a) (1 + b) (1 + a) − 1` of symmetric `a` and `b`, without the identity.
fn around(a: mat3x3<f32>, b: mat3x3<f32>) -> mat3x3<f32> {
    // Products of symmetric matrices: `a * b` is the transpose of the product in the row
    // and column order used here, `ab + ba` and `aba` are the same either way.
    let ab = a * b;
    return 2.0 * a + b + a * a + ab + transpose(ab) + ab * a;
}

fn to_matrix(t: array<f32, 6>) -> mat3x3<f32> {
    return mat3x3<f32>(
        vec3<f32>(t[0], t[3], t[5]),
        vec3<f32>(t[3], t[1], t[4]),
        vec3<f32>(t[5], t[4], t[2]),
    );
}

fn to_vector(m: mat3x3<f32>) -> array<f32, 6> {
    return array<f32, 6>(m[0][0], m[1][1], m[2][2], m[0][1], m[1][2], m[2][0]);
}

// The processor's finite-strain J2 law (`FiniteStrainJ2`) of the Green-Lagrange strain
// (engineering components): the second Piola-Kirchhoff stress. With the stretch
// `U = √(1 + 2E) = 1 + A`, `U⁻¹ = 1 + A'` and the state's `P = C_p⁻¹ − 1`: the trial
// `b_e − 1 = around(A, P)`, its logarithmic strain, the return mapping on the Kirchhoff
// stress, `C_p⁻¹ − 1 = around(A', exp(2 ε_e) − 1)` and `S = U⁻¹ τ U⁻¹`. Every matrix is
// a difference to the identity, so that single precision holds the small strains.
fn finite_j2(m: u32, strain: array<f32, 6>, ip: u32, c: u32, e: u32) -> array<f32, 6> {
    let at = ip * STATE;
    let two_e = mat3x3<f32>(
        vec3<f32>(2.0 * strain[0], strain[3], strain[5]),
        vec3<f32>(strain[3], 2.0 * strain[1], strain[4]),
        vec3<f32>(strain[5], strain[4], 2.0 * strain[2]),
    );
    let stretch = eigen(two_e);
    var fa = vec3<f32>();
    var fi = vec3<f32>();
    for (var k = 0u; k < 3u; k++) {
        let l = stretch.values[k];
        let s = sqrt(max(1.0 + l, 1e-30));
        fa[k] = l / (1.0 + s);
        fi[k] = -fa[k] / s;
    }
    let a = rebuild(stretch.vectors, fa);
    let ai = rebuild(stretch.vectors, fi);
    var plastic = array<f32, 6>();
    for (var k = 0u; k < 6u; k++) {
        plastic[k] = f32(history[(at + 1u + k) * c + e]);
    }
    let trial_b = eigen(around(a, to_matrix(plastic)));
    var logs = vec3<f32>();
    for (var k = 0u; k < 3u; k++) {
        logs[k] = 0.5 * log1p(trial_b.values[k]);
    }
    let trial = to_vector(rebuild(trial_b.vectors, logs));
    let r = radial(m, trial, ip, c, e);
    if r.flowed {
        var elastic = array<f32, 6>();
        for (var k = 0u; k < 6u; k++) {
            elastic[k] = trial[k] - r.dep[k];
        }
        let be = eigen(to_matrix(elastic));
        var exps = vec3<f32>();
        for (var k = 0u; k < 3u; k++) {
            exps[k] = expm1(2.0 * be.values[k]);
        }
        let cp = to_vector(around(ai, rebuild(be.vectors, exps)));
        for (var k = 0u; k < 6u; k++) {
            history[(at + 1u + k) * c + e] = f64(cp[k]);
        }
    }
    let tau = to_matrix(r.s);
    let ta = ai * tau;
    return to_vector(tau + ta + transpose(ta) + ta * ai);
}

// x^k for a small whole k ≥ 0.
fn whole_power(x: f32, k: u32) -> f32 {
    var p = 1.0;
    for (var i = 0u; i < k; i++) {
        p *= x;
    }
    return p;
}

// The hyperelastic law of the material block `m` (see `materials` in `gpu.rs`) for the
// Green-Lagrange strain: the second Piola-Kirchhoff stress, formed from `M = C − 1` as
// `hyperelastic` in `elements.rs` does, so that small strains keep their digits.
fn hyperelastic(m: u32, strain: array<f32, 6>) -> array<f32, 6> {
    let mx = mat3x3<f32>(
        vec3<f32>(2.0 * strain[0], strain[3], strain[5]),
        vec3<f32>(strain[3], 2.0 * strain[1], strain[4]),
        vec3<f32>(strain[5], strain[4], 2.0 * strain[2]),
    );
    let mm = mx * mx;
    let tr = mx[0][0] + mx[1][1] + mx[2][2];
    let tr2 = mm[0][0] + mm[1][1] + mm[2][2];
    let i2 = 0.5 * (tr * tr - tr2);
    let det_m = determinant(mx);
    let dm1 = tr + i2 + det_m;
    let det = 1.0 + dm1;
    let j = sqrt(det);
    let jm1 = dm1 / (1.0 + j);
    var dudj = 0.0;
    if materials[m + 1u] == 0.0 {
        for (var k = 0u; k < 3u; k++) {
            let power = 2u * k + 2u;
            dudj += materials[m + 2u + k] * f32(power) * whole_power(jm1, power - 1u);
        }
    } else {
        dudj = materials[m + 2u] * jm1 * (j + 1.0) / j;
    }
    let pressure = j * dudj;
    let log_j = 0.5 * log1p(dm1);
    let identity = mat3x3<f32>(
        vec3<f32>(1.0, 0.0, 0.0),
        vec3<f32>(0.0, 1.0, 0.0),
        vec3<f32>(0.0, 0.0, 1.0),
    );
    // C⁻¹ − 1.
    let n = (1.0 / det) * (mm - (1.0 + tr) * mx - det_m * identity);
    let model = u32(materials[m + 5u]);
    let terms = u32(materials[m + 6u]);
    var s = mat3x3<f32>();
    if model == 2u {
        // Ogden: Kirchhoff principal stresses on the principal stretches.
        let eig = eigen(mx);
        var tau = vec3<f32>(pressure, pressure, pressure);
        for (var t = 0u; t < terms; t++) {
            let mu = materials[m + 7u + 2u * t];
            let alpha = materials[m + 8u + 2u * t];
            var p = vec3<f32>();
            for (var a = 0u; a < 3u; a++) {
                p[a] = expm1(alpha * (0.5 * log1p(eig.values[a]) - log_j / 3.0));
            }
            let mean = (p.x + p.y + p.z) / 3.0;
            tau += 2.0 * mu / alpha * (p - vec3<f32>(mean, mean, mean));
        }
        s = rebuild(eig.vectors, tau / (vec3<f32>(1.0, 1.0, 1.0) + eig.values));
    } else {
        let j23m1 = expm1(-2.0 / 3.0 * log_j);
        let j43m1 = expm1(-4.0 / 3.0 * log_j);
        let j23 = 1.0 + j23m1;
        // Ī1 − 3 and Ī2 − 3.
        let x = 3.0 * j23m1 + j23 * tr;
        let y = 3.0 * j43m1 + (1.0 + j43m1) * (2.0 * tr + i2);
        var w1 = 0.0;
        var w2 = 0.0;
        if model == 0u {
            for (var t = 0u; t < terms; t++) {
                let i = u32(materials[m + 7u + 3u * t]);
                let k = u32(materials[m + 8u + 3u * t]);
                let c = materials[m + 9u + 3u * t];
                if i > 0u {
                    w1 += c * f32(i) * whole_power(x, i - 1u) * whole_power(y, k);
                }
                if k > 0u {
                    w2 += c * f32(k) * whole_power(x, i) * whole_power(y, k - 1u);
                }
            }
        } else {
            // Arruda-Boyce: the inverse Langevin series.
            let coefficients = array<f32, 5>(0.5, 0.05, 0.0104761905, 0.00271428571, 0.000770315399);
            let mu = materials[m + 7u];
            let locking2 = materials[m + 8u] * materials[m + 8u];
            let i1 = 3.0 + x;
            var factor = mu;
            for (var k = 0u; k < 5u; k++) {
                w1 += factor * coefficients[k] * f32(k + 1u) * whole_power(i1, k);
                factor /= locking2;
            }
        }
        // S̄ = a 1 − b C; S = J^(−2/3) (S̄ − ⅓ (S̄ : C) C⁻¹) + p C⁻¹.
        let a = 2.0 * (w1 + (3.0 + x) * w2);
        let b = 2.0 * w2 * j23;
        let share = b / 3.0 * (tr + tr2) - (a - b) * tr / 3.0;
        s = j23 * (-(a - b) * n - b * mx + share * (identity + n)) + pressure * (identity + n);
    }
    return to_vector(s);
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
        } else if kind == 2u {
            s = finite_j2(m, strain, ip, c, e);
        } else if kind == 3u {
            s = hyperelastic(m, strain);
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
