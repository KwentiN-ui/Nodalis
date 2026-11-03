struct Globals {
    view_proj: mat4x4<f32>,
    // Directions towards the three camera lights, in world space.
    lights: array<vec4<f32>, 3>,
    // Direction towards the viewer (orthographic, so the same everywhere).
    view_dir: vec4<f32>,
    background_top: vec4<f32>,
    background_bottom: vec4<f32>,
    // x, y: target size in pixels; z: width of wide edges in pixels; w: their pull towards the
    // viewer, half their width.
    edge: vec4<f32>,
    // x: number of contour bands, 0 to show part colours.
    contour: vec4<f32>,
    // Section view plane: unit normal in xyz, normal · point in w.
    clip: vec4<f32>,
    // x: 1 while the section view is on, y: tolerance behind the plane in world units.
    clip_options: vec4<f32>,
    // Colour of the wireframe of the undeformed shape in rgb.
    wireframe: vec4<f32>,
    // x: LIC source, 0 none, 1 vector, 2 tensor; y: principal value of a tensor, 0 to 2 from
    // the largest, 3 the largest in magnitude; z: grain size in world units; w: size of a pixel in world units.
    lic: vec4<f32>,
    // x: streak length in pixels, y: contrast, z: mix.
    lic_params: vec4<f32>,
    palette: array<vec4<f32>, 26>,
};

@group(0) @binding(0) var<uniform> globals: Globals;

// Like VTK, all colours and lighting live in display (sRGB) space: the target is a plain UNORM
// texture that the GUI shows as is, so shaded colours are written without any encoding.

struct BackgroundOut {
    @builtin(position) position: vec4<f32>,
    @location(0) t: f32,
};

@vertex
fn vs_background(@builtin(vertex_index) index: u32) -> BackgroundOut {
    let uv = vec2<f32>(f32((index << 1u) & 2u), f32(index & 2u));
    var out: BackgroundOut;
    out.position = vec4<f32>(uv * 2.0 - 1.0, 1.0, 1.0);
    out.t = uv.y * 0.5;
    return out;
}

@fragment
fn fs_background(in: BackgroundOut) -> @location(0) vec4<f32> {
    return vec4<f32>(mix(globals.background_bottom, globals.background_top, clamp(in.t, 0.0, 1.0)).rgb, 1.0);
}

struct SurfaceIn {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec3<f32>,
    @location(3) scalar: f32,
};

struct SurfaceOut {
    @builtin(position) position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) color: vec3<f32>,
    @location(2) scalar: f32,
    @location(3) world: vec3<f32>,
};

// Everything on the cut-off side of the section plane is not drawn.
fn clipped(world: vec3<f32>) -> bool {
    return globals.clip_options.x > 0.0
        && dot(globals.clip.xyz, world) - globals.clip.w < -globals.clip_options.y;
}

// Colour without a result value: light grey.
const NO_VALUE_COLOR: vec3<f32> = vec3<f32>(0.75, 0.75, 0.75);

// Material properties and light intensity.
const AMBIENT: f32 = 0.6;
const DIFFUSE: f32 = 0.6;
const SPECULAR: f32 = 0.6;
const SPECULAR_POWER: f32 = 100.0;
const LIGHT_INTENSITY: f32 = 0.4;

@vertex
fn vs_surface(in: SurfaceIn) -> SurfaceOut {
    var out: SurfaceOut;
    out.position = globals.view_proj * vec4<f32>(in.position, 1.0);
    out.normal = in.normal;
    out.color = in.color;
    out.scalar = in.scalar;
    out.world = in.position;
    return out;
}

// Discrete band of a normalized value; the top value 1.0 belongs to the last band.
fn contour_color(t: f32) -> vec3<f32> {
    let levels = globals.contour.x;
    if t < 0.0 {
        return NO_VALUE_COLOR;
    }
    let band = u32(clamp(floor(t * levels), 0.0, levels - 1.0));
    return globals.palette[band].rgb;
}

@fragment
fn fs_surface(in: SurfaceOut) -> @location(0) vec4<f32> {
    if clipped(in.world) {
        discard;
    }
    return shade(in.normal, in.color, in.scalar);
}

// The lit colour of a surface.
fn shade(normal: vec3<f32>, part_color: vec3<f32>, scalar: f32) -> vec4<f32> {
    var color = part_color;
    if globals.contour.x > 0.0 {
        color = contour_color(scalar);
    }
    // Two-sided lighting: back faces (inside of open shells) are lit like front faces.
    var n = normalize(normal);
    let v = globals.view_dir.xyz;
    if dot(n, v) < 0.0 {
        n = -n;
    }
    var diffuse = 0.0;
    var specular = 0.0;
    for (var i = 0; i < 3; i++) {
        let l = globals.lights[i].xyz;
        diffuse += LIGHT_INTENSITY * max(dot(n, l), 0.0);
        let h = normalize(l + v);
        specular += LIGHT_INTENSITY * pow(max(dot(n, h), 0.0), SPECULAR_POWER);
    }
    let shaded = color * (AMBIENT + DIFFUSE * diffuse) + vec3<f32>(SPECULAR * specular);
    return vec4<f32>(clamp(shaded, vec3<f32>(0.0), vec3<f32>(1.0)), 1.0);
}

// Surfaces for line integral convolution: they also carry the vector source of their vertices
// (Directions of mesh.rs), a vector or a tensor xx, yy, zz in `a` and xy, yz, zx in `b`.
struct LicSurfaceIn {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec3<f32>,
    @location(3) scalar: f32,
    @location(4) a: vec3<f32>,
    @location(5) b: vec3<f32>,
};

struct LicSurfaceOut {
    @builtin(position) position: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) color: vec3<f32>,
    @location(2) scalar: f32,
    @location(3) world: vec3<f32>,
    @location(4) a: vec3<f32>,
    @location(5) b: vec3<f32>,
};

struct LicFragment {
    @location(0) color: vec4<f32>,
    // xy: unit direction on the screen in pixels, y down; z: how much LIC shows, 0 to 1;
    // w: the noise.
    @location(1) lic: vec4<f32>,
};

@vertex
fn vs_surface_lic(in: LicSurfaceIn) -> LicSurfaceOut {
    var out: LicSurfaceOut;
    out.position = globals.view_proj * vec4<f32>(in.position, 1.0);
    out.normal = in.normal;
    out.color = in.color;
    out.scalar = in.scalar;
    out.world = in.position;
    out.a = in.a;
    out.b = in.b;
    return out;
}

@fragment
fn fs_surface_lic(in: LicSurfaceOut) -> LicFragment {
    if clipped(in.world) {
        discard;
    }
    var out: LicFragment;
    out.color = shade(in.normal, in.color, in.scalar);
    out.lic = lic_data(in.world, normalize(in.normal), in.a, in.b);
    return out;
}

const PI: f32 = 3.14159265;

// Principal values of the symmetric tensor, largest first, in closed form like
// principal_values of plx-results.
fn principal_values(d: vec3<f32>, o: vec3<f32>) -> vec3<f32> {
    let mean = (d.x + d.y + d.z) / 3.0;
    let c = d - vec3<f32>(mean);
    let off = dot(o, o);
    let p = sqrt((dot(c, c) + 2.0 * off) / 6.0);
    if p <= 1e-12 {
        return vec3<f32>(mean);
    }
    // o = (xy, yz, zx)
    let det = c.x * (c.y * c.z - o.y * o.y) - o.x * (o.x * c.z - o.y * o.z)
        + o.z * (o.x * o.y - c.y * o.z);
    let r = clamp(det / (2.0 * p * p * p), -1.0, 1.0);
    let phi = acos(r) / 3.0;
    let p1 = mean + 2.0 * p * cos(phi);
    let p3 = mean + 2.0 * p * cos(phi + 2.0 * PI / 3.0);
    return vec3<f32>(p1, 3.0 * mean - p1 - p3, p3);
}

// Eigenvector of a simple eigenvalue: the longest cross product of two rows of T - lambda I.
fn eigenvector(d: vec3<f32>, o: vec3<f32>, lambda: f32) -> vec3<f32> {
    let r0 = vec3<f32>(d.x - lambda, o.x, o.z);
    let r1 = vec3<f32>(o.x, d.y - lambda, o.y);
    let r2 = vec3<f32>(o.z, o.y, d.z - lambda);
    let c0 = cross(r0, r1);
    let c1 = cross(r0, r2);
    let c2 = cross(r1, r2);
    var best = c0;
    if dot(c1, c1) > dot(best, best) {
        best = c1;
    }
    if dot(c2, c2) > dot(best, best) {
        best = c2;
    }
    let len = length(best);
    if len <= 0.0 {
        return vec3<f32>(0.0);
    }
    return best / len;
}

// The direction of the vector source in world space and where it is defined well enough to
// show streaks, 0 to 1. Values are scaled so that the largest magnitude is about 1.
fn source_direction(a: vec3<f32>, b: vec3<f32>) -> vec4<f32> {
    if globals.lic.x < 1.5 {
        let len = length(a);
        if len <= 0.0 {
            return vec4<f32>(0.0);
        }
        return vec4<f32>(a / len, smoothstep(0.002, 0.01, len));
    }
    let values = principal_values(a, b);
    let scale = max(abs(values.x), max(abs(values.y), abs(values.z)));
    var which = u32(globals.lic.y);
    // The largest in magnitude: tension where it pulls, compression where it pushes. Where
    // both are equally large the direction turns by 90 degrees, so the streaks fade there.
    var switch_gap = 1e30;
    if which == 3u {
        which = select(0u, 2u, abs(values.z) > abs(values.x));
        switch_gap = abs(abs(values.x) - abs(values.z));
    }
    var gap = values.x - values.y;
    if which == 1u {
        gap = min(values.x - values.y, values.y - values.z);
    } else if which == 2u {
        gap = values.y - values.z;
    }
    gap = min(gap, switch_gap);
    // Nearly repeated principal values leave the direction open, as in unloaded or
    // hydrostatic regions.
    let mask = smoothstep(0.002, 0.01, scale) * smoothstep(0.02, 0.08, gap / max(scale, 1e-12));
    if mask <= 0.0 {
        return vec4<f32>(0.0);
    }
    return vec4<f32>(eigenvector(a, b, values[which]), mask);
}

// A value from 0 to 1 per cube of the noise grid.
fn hash(cell: vec3<i32>) -> f32 {
    var v = bitcast<vec3<u32>>(cell) * 1664525u + vec3<u32>(1013904223u);
    v.x += v.y * v.z;
    v.y += v.z * v.x;
    v.z += v.x * v.y;
    v = v ^ (v >> vec3<u32>(16u));
    v.x += v.y * v.z;
    v.y += v.z * v.x;
    v.z += v.x * v.y;
    return f32(v.x >> 8u) / 16777215.0;
}

fn lic_data(world: vec3<f32>, normal: vec3<f32>, a: vec3<f32>, b: vec3<f32>) -> vec4<f32> {
    let source = source_direction(a, b);
    // The streaks follow the direction within the surface; across it there is nothing to
    // follow, so a direction along the normal shows none.
    let along = source.xyz - dot(source.xyz, normal) * normal;
    let v = globals.view_dir.xyz;
    // Seen edge on, the direction shrinks to a point on the screen.
    let seen = length(along - dot(along, v) * v);
    let mask = source.w * smoothstep(0.1, 0.3, seen);
    let size = globals.edge.xy;
    let projected = globals.view_proj * vec4<f32>(along, 0.0);
    let screen = vec2<f32>(projected.x * size.x, -projected.y * size.y);
    let len = length(screen);
    // Noise fixed on the model, so that it does not swim when the view turns.
    let grain = max(globals.lic.z, globals.lic.w);
    let noise = hash(vec3<i32>(floor(world / grain)));
    if mask <= 0.0 || len <= 0.0 {
        return vec4<f32>(0.0, 0.0, 0.0, noise);
    }
    return vec4<f32>(screen / len, mask, noise);
}

struct EdgeOut {
    @builtin(position) position: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) world: vec3<f32>,
};

// Edges lie exactly on surface triangles; the surfaces are pushed back behind them by a
// slope-scaled depth bias (EDGE_OFFSET_PX of renderer.rs), so they pass the depth test.
fn edge_vertex(position: vec3<f32>, color: vec4<f32>) -> EdgeOut {
    var out: EdgeOut;
    out.position = globals.view_proj * vec4<f32>(position, 1.0);
    out.color = color;
    out.world = position;
    return out;
}

@vertex
fn vs_edge(in: SurfaceIn) -> EdgeOut {
    // Line vertices carry their opacity in the scalar slot.
    return edge_vertex(in.position, vec4<f32>(in.color, in.scalar));
}

// The wireframe of the undeformed shape takes its colour from the theme.
@vertex
fn vs_wireframe(in: SurfaceIn) -> EdgeOut {
    return edge_vertex(in.position, vec4<f32>(globals.wireframe.rgb, in.scalar));
}

// MESH_EDGE_OPACITY of fe.rs.
const MESH_EDGE_OPACITY: f32 = 0.4;

// Mesh edges share the vertices of the surfaces, whose colours they do not take.
@vertex
fn vs_mesh_edge(in: SurfaceIn) -> EdgeOut {
    return edge_vertex(in.position, vec4<f32>(0.0, 0.0, 0.0, MESH_EDGE_OPACITY));
}

// One corner of the quad of a wide line segment from `a` to `b`; corners 0 and 2 lie at `a`,
// 1 and 3 at `b`, on either side of the line, drawn as triangles 0-1-2 and 2-1-3.
@vertex
fn vs_wide_edge(
    @builtin(vertex_index) index: u32,
    @location(0) a: vec3<f32>,
    @location(1) color: vec3<f32>,
    @location(2) b: vec3<f32>,
) -> EdgeOut {
    let corner = array<u32, 6>(0u, 1u, 2u, 2u, 1u, 3u)[index];
    var ends = array<vec4<f32>, 2>(
        globals.view_proj * vec4<f32>(a, 1.0),
        globals.view_proj * vec4<f32>(b, 1.0),
    );
    let size = globals.edge.xy;
    let screen_a = ends[0].xy / ends[0].w * size;
    let screen_b = ends[1].xy / ends[1].w * size;
    var along = screen_b - screen_a;
    if length(along) < 1e-6 {
        along = vec2<f32>(1.0, 0.0);
    }
    along = normalize(along);
    let across = vec2<f32>(-along.y, along.x);
    let end = corner & 1u;
    let side = f32(corner >> 1u) * 2.0 - 1.0;
    let forward = f32(end) * 2.0 - 1.0;
    // Half the width to each side and beyond each end, so that segments join without gaps;
    // NDC spans two units across the target.
    let offset = (across * side + along * forward) * globals.edge.z / size;
    var out: EdgeOut;
    out.position = ends[end];
    // The quad reaches half its width into the faces beside the line, further than the depth
    // bias of a face sloping away from the viewer covers.
    out.position.z -= globals.edge.w * out.position.w;
    out.position = vec4<f32>(out.position.xy + offset * out.position.w, out.position.zw);
    out.color = vec4<f32>(color, 1.0);
    out.world = select(a, b, end == 1u);
    return out;
}

@fragment
fn fs_edge(in: EdgeOut) -> @location(0) vec4<f32> {
    if clipped(in.world) {
        discard;
    }
    return in.color;
}

@group(1) @binding(0) var scene_texture: texture_2d<f32>;
@group(1) @binding(1) var lic_texture: texture_2d<f32>;

@vertex
fn vs_lic(@builtin(vertex_index) index: u32) -> @builtin(position) vec4<f32> {
    let uv = vec2<f32>(f32((index << 1u) & 2u), f32(index & 2u));
    return vec4<f32>(uv * 2.0 - 1.0, 0.0, 1.0);
}

// Line integral convolution: the noise of the surfaces averaged along the streak through the
// pixel, a step of one pixel at a time to either side, weighted less towards its ends. The
// direction has no sign, so each step keeps the way of the one before. A streak ends where
// LIC does not show, such as at the outline of a part.
@fragment
fn fs_lic(@builtin(position) position: vec4<f32>) -> @location(0) vec4<f32> {
    let pixel = vec2<i32>(position.xy);
    let scene = textureLoad(scene_texture, pixel, 0);
    let centre = textureLoad(lic_texture, pixel, 0);
    if centre.z <= 0.001 {
        return scene;
    }
    let size = vec2<i32>(textureDimensions(lic_texture));
    let steps = i32(globals.lic_params.x);
    var sum = centre.w;
    var weight = 1.0;
    for (var side = 0; side < 2; side++) {
        var p = position.xy;
        var d = centre.xy * (f32(side) * 2.0 - 1.0);
        for (var i = 1; i <= steps; i++) {
            p += d;
            let q = vec2<i32>(floor(p));
            if any(q < vec2<i32>(0)) || any(q >= size) {
                break;
            }
            let here = textureLoad(lic_texture, q, 0);
            let len = length(here.xy);
            if here.z <= 0.001 || len <= 0.0 {
                break;
            }
            var next = here.xy / len;
            if dot(next, d) < 0.0 {
                next = -next;
            }
            d = next;
            let w = 1.0 - f32(i) / f32(steps + 1);
            sum += here.w * w;
            weight += w;
        }
    }
    let streak = clamp((sum / weight - 0.5) * globals.lic_params.y + 0.5, 0.0, 1.0);
    let mix_share = globals.lic_params.z * centre.z;
    let brightness = 1.0 - mix_share + mix_share * (0.3 + 1.2 * streak);
    return vec4<f32>(clamp(scene.rgb * brightness, vec3<f32>(0.0), vec3<f32>(1.0)), scene.a);
}
