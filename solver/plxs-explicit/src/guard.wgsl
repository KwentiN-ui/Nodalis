// The watch over the stable time step under large deformations (`guard.rs`): per element
// the shortest squared distance between two of its nodes, compared with the reference of
// the last check; the smallest ratio either way over all elements, as the bits of a
// positive f32 (which order like the numbers), by atomic minima.

struct GuardDims {
    count: u32,
    groups_x: u32,
    pad0: u32,
    pad1: u32,
}

@group(0) @binding(0) var<uniform> dims: GuardDims;
@group(0) @binding(1) var<storage, read> offsets: array<u32>;
// Three per node of an element.
@group(0) @binding(2) var<storage, read> dofs: array<u32>;
@group(0) @binding(3) var<storage, read> coordinates: array<f64>;
@group(0) @binding(4) var<storage, read> u: array<f64>;
@group(0) @binding(5) var<storage, read> reference: array<f32>;
@group(0) @binding(6) var<storage, read_write> result: array<atomic<u32>>;

@compute @workgroup_size(64)
fn guard(@builtin(workgroup_id) wid: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let e = (wid.y * dims.groups_x + wid.x) * 64u + li;
    if e >= dims.count {
        return;
    }
    let first = offsets[e];
    let last = offsets[e + 1u];
    var shortest = 3.4e38f;
    for (var a = first; a < last; a++) {
        for (var b = a + 1u; b < last; b++) {
            var l = 0.0f;
            for (var k = 0u; k < 3u; k++) {
                let d = (coordinates[3u * b + k] - coordinates[3u * a + k])
                    + (u[dofs[3u * b + k]] - u[dofs[3u * a + k]]);
                let s = f32(d);
                l += s * s;
            }
            shortest = min(shortest, l);
        }
    }
    let then = reference[e];
    if then <= 0.0f {
        return;
    }
    var r = 0.0f;
    if shortest > 0.0f && shortest < 3.4e38f {
        r = min(shortest / then, then / shortest);
    }
    atomicMin(&result[0], bitcast<u32>(r));
}
