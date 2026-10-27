//! SPIKE-002 photoreal pass: pure preparation and shaders.
//!
//! No GL calls occur here, so host tests prove asset/frame compatibility,
//! ASTC capability gating, batch boundaries and memory accounting. The plan
//! maps to one texture upload per asset texture, one static vertex and one
//! static index buffer, and one `glDrawElements` per mesh, all inside the S4
//! default framebuffer (one render target, frame-start clear, end-of-frame
//! depth invalidate: SPIKE-001c).
//!
//! Frame order (linux.rs): terrain (depth tested and written), sky (only
//! where nothing was drawn: depth LEQUAL at the far plane), decals and
//! lights (painter's order, no depth test, like the S4 ground), aircraft
//! (depth tested), HUD.
//!
//! Lighting is a fixed afternoon sun. Textures are sRGB, shading is done in
//! linear light and encoded to the (non-sRGB) scanout buffer with gamma
//! 1/2.2 at the end of every fragment shader. Water is wherever the ground
//! texture's alpha is below ~0.7 (the converter writes the 3DEP
//! topobathymetric land/water mask and shore depth into alpha).

use crate::camera::{Pose, View};
use crate::math::M4;
use crate::world::WorldDraw;
use gx_world::{Joint, JointDriver, JointKind, TextureEncoding, VisualAsset, VisualKind, VisualLimits, WorldAsset, MAX_JOINTS};

pub const ASTC_EXTENSION: &str = "GL_KHR_texture_compression_astc_ldr";
pub const GL_COMPRESSED_SRGB8_ALPHA8_ASTC_6X6_KHR: u32 = 0x93D4;
pub const GL_SRGB8_ALPHA8: u32 = 0x8C43;

/// Sun direction (ENU, unit): azimuth 225 deg (south-west), elevation 42 deg.
pub const SUN_ENU: [f32; 3] = [-0.525_5, -0.525_5, 0.669_1];

/// Shared fragment-shader code: sky radiance, aerial perspective, encode.
/// Kept cheap for the FD702: integer powers by repeated squaring, sqrt
/// (gamma 2.0) instead of pow(1/2.2) for the output encode.
const COMMON: &str = "
uniform vec3 u_sun;
const vec3 HORIZON = vec3(0.60, 0.70, 0.82);
const vec3 ZENITH = vec3(0.09, 0.22, 0.55);
const vec3 SUNCOL = vec3(1.00, 0.93, 0.80);
vec3 sky(vec3 d) {
    float s = max(dot(d, u_sun), 0.0);
    float s2 = s * s; float s4 = s2 * s2; float s8 = s4 * s4; float s64 = s8 * s8; s64 *= s64; s64 *= s64;
    vec3 glow = SUNCOL * (0.22 * s8 + 0.6 * s64);
    if (d.z < 0.0) {
        return mix(HORIZON, vec3(0.42, 0.47, 0.50), min(-d.z * 3.0, 1.0)) + glow * 0.5;
    }
    vec3 c = mix(HORIZON, ZENITH, sqrt(d.z));
    return c + glow + SUNCOL * smoothstep(0.99985, 0.99995, s) * 6.0;
}
vec3 aerial(vec3 c, float dist, vec3 dir) {
    float f = 1.0 - exp(-dist * (1.0 / 32000.0));
    float s = max(dot(dir, u_sun), 0.0); float s2 = s * s;
    vec3 h = HORIZON + SUNCOL * (0.28 * s2 * s2 * s2);
    return mix(c, h, f);
}
vec3 encode(vec3 c) {
    c = c * 1.08;
    c = c / (1.0 + 0.12 * c);
    return sqrt(clamp(c, 0.0, 1.0));
}
";

/// `vec4` uniforms of the joint array: three affine rows per joint, joint 0
/// (static) included. The shader's array size must match.
pub const JOINT_ROWS: usize = 3 * (MAX_JOINTS + 1);

pub const PHOTO_VERTEX_SHADER: &str = "#version 300 es
uniform mat4 u_mvp;
uniform mat4 u_rel;
uniform mat3 u_normal;
// Rigid joints (GXVISUAL 1.2, or the 1.1 hinges): three affine rows each,
// composed on the CPU (joint_rows); joint 0 is the identity.
uniform vec4 u_joints[195];
layout(location=0) in vec3 a_pos;
layout(location=1) in vec3 a_normal;
layout(location=2) in vec2 a_uv;
layout(location=3) in float a_animation;
out vec3 v_rel;
out vec3 v_normal;
out vec2 v_uv;
void main() {
    int j = 3 * int(a_animation + 0.5);
    vec4 r0 = u_joints[j]; vec4 r1 = u_joints[j + 1]; vec4 r2 = u_joints[j + 2];
    vec4 p = vec4(a_pos, 1.0);
    vec3 pos = vec3(dot(r0, p), dot(r1, p), dot(r2, p));
    vec3 norm = vec3(dot(r0.xyz, a_normal), dot(r1.xyz, a_normal), dot(r2.xyz, a_normal));
    v_rel = (u_rel * vec4(pos, 1.0)).xyz;
    v_normal = u_normal * norm;
    v_uv = a_uv;
    gl_Position = u_mvp * vec4(pos, 1.0);
}
";

/// Fragment shader for terrain, decals and the aircraft.
pub fn photo_fragment_shader() -> String {
    format!(
        "#version 300 es
precision highp float;
{COMMON}
uniform sampler2D u_base_color;
uniform vec4 u_base_color_factor;
uniform float u_roughness;
uniform float u_gloss;
uniform vec3 u_eye;
uniform float u_time;
in vec3 v_rel;
in vec3 v_normal;
in vec2 v_uv;
out vec4 o_col;
vec2 wave(vec2 p, vec2 dir, float len, float slope, float speed) {{
    float k = 6.2831853 / len;
    return dir * (slope * cos(k * dot(dir, p) - speed * k * u_time));
}}
void main() {{
    vec4 t = texture(u_base_color, v_uv);
    vec3 albedo = t.rgb * u_base_color_factor.rgb;
    float dist = max(length(v_rel), 0.01);
    vec3 vdir = v_rel / dist;
    vec3 n = normalize(v_normal);
    float ndl = max(dot(n, u_sun), 0.0);
    vec3 col = albedo * (vec3(0.40, 0.45, 0.52) + SUNCOL * 0.92 * ndl);
    if (u_gloss > 0.0) {{
        vec3 h = normalize(u_sun - vdir);
        col += SUNCOL * u_gloss * pow(max(dot(n, h), 0.0), mix(160.0, 12.0, u_roughness));
        col += sky(reflect(vdir, n)) * u_gloss * 0.25;
    }}
    float land = smoothstep(0.64, 0.80, t.a);
    if (land < 1.0) {{
        vec3 wn = vec3(0.0, 0.0, 1.0);
        if (dist < 6000.0) {{
            // Waves fade out with distance (no aliasing, no cost far away).
            vec2 p = v_rel.xy + u_eye.xy;
            float amp = exp(-dist * (1.0 / 1800.0));
            vec2 g = wave(p, vec2(0.80, 0.60), 31.0, 0.050, 2.2)
                   + wave(p, vec2(0.28, 0.96), 13.0, 0.045, 1.5)
                   + wave(p, vec2(0.96, -0.28), 7.0, 0.040, 1.1)
                   + wave(p, vec2(-0.55, 0.83), 3.1, 0.035, 0.7);
            wn = normalize(vec3(-g * amp, 1.0));
        }}
        vec3 r = reflect(vdir, wn);
        r.z = abs(r.z);
        float m = 1.0 - max(dot(-vdir, wn), 0.0);
        float m2 = m * m;
        float fres = 0.02 + 0.98 * m2 * m2 * m;
        float shore = clamp(t.a * (1.0 / 0.6), 0.0, 1.0);
        vec3 body = mix(vec3(0.012, 0.045, 0.060), vec3(0.050, 0.120, 0.115), shore);
        body = mix(body, albedo * 0.7, 0.30 * shore);
        float sp = max(dot(r, u_sun), 0.0);
        float sp2 = sp * sp; float sp4 = sp2 * sp2; float sp16 = sp4 * sp4; sp16 *= sp16;
        float sp256 = sp16 * sp16; sp256 *= sp256;
        vec3 w = mix(body, sky(r), fres) + SUNCOL * 5.0 * sp256 * sp16 * sp16 * sp16;
        col = mix(w, col, land);
    }}
    o_col = vec4(encode(aerial(col, dist, vdir)), 1.0);
}}
"
    )
}

/// Full-screen sky: one triangle from `gl_VertexID`, no vertex buffer.
pub const SKY_VERTEX_SHADER: &str = "#version 300 es
out vec2 v_ndc;
void main() {
    vec2 p = vec2(gl_VertexID == 1 ? 3.0 : -1.0, gl_VertexID == 2 ? 3.0 : -1.0);
    v_ndc = p;
    gl_Position = vec4(p, 1.0, 1.0);
}
";

pub fn sky_fragment_shader() -> String {
    format!(
        "#version 300 es
precision highp float;
{COMMON}
uniform vec3 u_right;
uniform vec3 u_up;
uniform vec3 u_forward;
uniform vec2 u_tan;
in vec2 v_ndc;
out vec4 o_col;
void main() {{
    vec3 d = normalize(u_forward + v_ndc.x * u_tan.x * u_right + v_ndc.y * u_tan.y * u_up);
    o_col = vec4(encode(sky(d)), 1.0);
}}
"
    )
}

/// What the photo pass needs per frame, beyond the S4 [`WorldDraw`].
#[derive(Clone, Copy, Debug)]
pub struct PhotoDraw {
    /// Clear colour, ground MVP, aircraft MVP (`None` in the cockpit), light size.
    pub world: WorldDraw,
    /// Absolute ENU -> eye-relative ENU (a translation).
    pub ground_rel: [f32; 16],
    /// Body FRD -> eye-relative ENU.
    pub aircraft_rel: [f32; 16],
    /// Body FRD -> ENU rotation, column-major 3x3 (the aircraft normal matrix).
    pub aircraft_rot: [f32; 9],
    pub eye: [f32; 3],
    pub right: [f32; 3],
    pub up: [f32; 3],
    pub forward: [f32; 3],
    /// tan(half horizontal fov), tan(half vertical fov).
    pub tan_half: [f32; 2],
    /// Seconds since the photo scene was loaded (water animation).
    pub time_s: f32,
    /// Joint driver values, indexed by `JointDriver as usize - 1`: aileron,
    /// elevator, rudder (-1..1), flap, speedbrake, gear (0..1).
    pub drivers: [f32; 6],
}

impl PhotoDraw {
    /// `drivers` as in [`PhotoDraw::drivers`].
    pub fn new(world: WorldDraw, view: &View, pose: &Pose, time_s: f64, drivers: [f64; 6]) -> Self {
        let mut t = M4::identity();
        t.0[3] = [-view.eye[0], -view.eye[1], -view.eye[2], 1.0];
        let r = pose.rotation();
        let mut rot = [0.0f32; 9];
        for c in 0..3 {
            for row in 0..3 {
                rot[c * 3 + row] = r[row][c] as f32;
            }
        }
        let tan_v = (view.mode.vfov_deg().to_radians() / 2.0).tan();
        let aspect = f64::from(crate::scene::WIDTH / crate::scene::HEIGHT);
        let f = |v: [f64; 3]| v.map(|x| x as f32);
        Self {
            world,
            ground_rel: t.to_f32(),
            aircraft_rel: pose.model_relative(view.eye).to_f32(),
            aircraft_rot: rot,
            eye: f(view.eye),
            right: f(view.right),
            up: f(view.up),
            forward: f(view.forward),
            tan_half: [(tan_v * aspect) as f32, tan_v as f32],
            time_s: time_s as f32,
            drivers: sanitize_drivers(drivers),
        }
    }
}

/// Bounds untrusted/interpolated FDM animation values before GLES. A latest
/// snapshot fallback can itself contain a non-finite field, so clamp alone is
/// insufficient (`NaN.clamp` remains NaN). Surfaces are -1..1; flap,
/// speedbrake and gear 0..1.
pub fn sanitize_drivers(values: [f64; 6]) -> [f32; 6] {
    std::array::from_fn(|i| {
        let low = if i < 3 { -1.0 } else { 0.0 };
        if values[i].is_finite() { values[i].clamp(low, 1.0) as f32 } else { 0.0 }
    })
}

/// The GXVISUAL 1.1 bomb-5000 hinges (fitted to the static AI mesh) as
/// joints, so one shader path serves 1.1 and 1.2: right/left aileron,
/// right/left elevator, rudder; +-0.35 rad at full deflection with the
/// former shader's signs.
pub fn legacy_joints() -> Vec<Joint> {
    fn hinge(driver: JointDriver, center: [f32; 3], axis: [f32; 3], full_rad: f32) -> Joint {
        let n = axis.iter().map(|v| v * v).sum::<f32>().sqrt();
        let deg = full_rad.to_degrees();
        Joint {
            parent: 0,
            driver,
            kind: JointKind::Rotate,
            center,
            axis: axis.map(|v| v / n),
            table: vec![[-1.0, -deg], [1.0, deg]],
        }
    }
    vec![
        hinge(JointDriver::Aileron, [-2.2, 7.0, 1.1], [-3.7, 6.7, -0.4], -0.35),
        hinge(JointDriver::Aileron, [-2.2, -7.0, 1.1], [-3.7, -6.7, -0.4], -0.35),
        hinge(JointDriver::Elevator, [-11.35, 0.3, -4.05], [-3.3, 4.3, 0.25], 0.35),
        hinge(JointDriver::Elevator, [-11.35, -0.3, -4.05], [-3.3, -4.3, 0.25], -0.35),
        hinge(JointDriver::Rudder, [-10.25, 0.0, -1.0], [-2.25, 0.0, -3.0], -0.35),
    ]
}

/// Row-major 3x4 affine matrix.
pub type Affine = [[f32; 4]; 3];

pub const IDENTITY: Affine = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]];

fn compose(a: &Affine, b: &Affine) -> Affine {
    std::array::from_fn(|r| {
        std::array::from_fn(|c| {
            let v = a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c];
            if c == 3 { v + a[r][3] } else { v }
        })
    })
}

/// One joint's own transform at its driver value.
fn local(j: &Joint, value: f32) -> Affine {
    let out = j.evaluate(value);
    let a = j.axis;
    match j.kind {
        JointKind::Translate => {
            let mut m = IDENTITY;
            for (r, row) in m.iter_mut().enumerate() {
                row[3] = a[r] * out;
            }
            m
        }
        JointKind::Rotate => {
            // Rodrigues, right-handed about `axis`, through `center`.
            let (s, c) = out.to_radians().sin_cos();
            let t = 1.0 - c;
            let r: [[f32; 3]; 3] = [
                [c + a[0] * a[0] * t, a[0] * a[1] * t - a[2] * s, a[0] * a[2] * t + a[1] * s],
                [a[1] * a[0] * t + a[2] * s, c + a[1] * a[1] * t, a[1] * a[2] * t - a[0] * s],
                [a[2] * a[0] * t - a[1] * s, a[2] * a[1] * t + a[0] * s, c + a[2] * a[2] * t],
            ];
            let p = j.center;
            std::array::from_fn(|row| {
                let rp = r[row][0] * p[0] + r[row][1] * p[1] + r[row][2] * p[2];
                [r[row][0], r[row][1], r[row][2], p[row] - rp]
            })
        }
    }
}

/// Every joint's world transform (`world = parent * local`); index 0 is the
/// static identity, index `j` joint `j`.
pub fn joint_transforms(joints: &[Joint], drivers: &[f32; 6]) -> Vec<Affine> {
    let mut world: Vec<Affine> = Vec::with_capacity(joints.len() + 1);
    world.push(IDENTITY);
    for j in joints {
        let value = drivers[j.driver as usize - 1];
        let parent = world[j.parent as usize];
        world.push(compose(&parent, &local(j, value)));
    }
    world
}

/// The `u_joints` upload: [`joint_transforms`] as three rows per joint.
pub fn joint_rows(joints: &[Joint], drivers: &[f32; 6]) -> Vec<[f32; 4]> {
    joint_transforms(joints, drivers).into_iter().flatten().collect()
}

/// `m` applied to point `p`.
pub fn transform_point(m: &Affine, p: [f32; 3]) -> [f32; 3] {
    std::array::from_fn(|r| m[r][0] * p[0] + m[r][1] * p[1] + m[r][2] * p[2] + m[r][3])
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct TexturedVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub uv: [f32; 2],
    pub animation: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TextureUpload<'a> {
    pub id: u32,
    pub width: u32,
    pub height: u32,
    pub levels: u32,
    pub gl_internal_format: u32,
    pub compressed: bool,
    pub bytes: u64,
    /// Validated level payloads, largest first.
    pub mip_levels: &'a [Vec<u8>],
    /// Sampled with GL_REPEAT: a texture only Building meshes use (facade
    /// tiles in metre UVs). Everything else clamps to edge.
    pub repeat: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhotoBatch {
    pub kind: VisualKind,
    pub material_id: u32,
    pub texture_id: u32,
    /// First index (elements, not bytes) into the shared index buffer.
    pub first_index: u32,
    pub index_count: u32,
    pub base_color_factor: [f32; 4],
    pub roughness: f32,
    pub metallic: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PhotoMemory {
    pub texture_bytes: u64,
    pub vertex_bytes: u64,
    pub index_bytes: u64,
    /// Static bytes handed to GL (textures + vertices + indices); what the
    /// driver keeps resident is measured separately (DRM fdinfo).
    pub gpu_static_bytes: u64,
    /// Terrain, decal, building, aircraft.
    pub triangles: [u64; 4],
}

#[derive(Clone, Debug, PartialEq)]
pub struct PhotoPlan<'a> {
    pub textures: Vec<TextureUpload<'a>>,
    pub vertices: Vec<TexturedVertex>,
    pub indices: Vec<u32>,
    /// Draw order: terrain, then decals, then aircraft (stable within a kind).
    pub batches: Vec<PhotoBatch>,
    /// The asset's joints (1.2), or [`legacy_joints`] for 1.0/1.1.
    pub joints: Vec<Joint>,
    pub memory: PhotoMemory,
}

fn has_extension(extensions: &str, needle: &str) -> bool {
    extensions.split_ascii_whitespace().any(|token| token == needle)
}

fn kind_rank(k: VisualKind) -> u8 {
    match k {
        VisualKind::Terrain => 0,
        VisualKind::Decal => 1,
        VisualKind::Building => 2,
        VisualKind::Aircraft => 3,
    }
}

/// Validate and flatten a visual sidecar into GLES upload/draw records.
pub fn prepare_photo<'a>(
    world: &WorldAsset,
    visual: &'a VisualAsset,
    gl_extensions: &str,
    limits: VisualLimits,
) -> Result<PhotoPlan<'a>, String> {
    visual.validate(limits).map_err(|e| e.to_string())?;
    if !visual.matches_world(world) {
        return Err("visual/world frame identity mismatch".into());
    }
    let needs_astc = visual.textures.iter().any(|t| t.encoding == TextureEncoding::Astc6x6Srgb);
    if needs_astc && !has_extension(gl_extensions, ASTC_EXTENSION) {
        return Err(format!("ASTC texture requires {ASTC_EXTENSION}"));
    }
    let mut textures = visual
        .textures
        .iter()
        .map(|t| {
            let (gl_internal_format, compressed) = match t.encoding {
                TextureEncoding::Astc6x6Srgb => (GL_COMPRESSED_SRGB8_ALPHA8_ASTC_6X6_KHR, true),
                TextureEncoding::Rgba8Srgb => (GL_SRGB8_ALPHA8, false),
            };
            TextureUpload {
                id: t.id,
                width: t.width,
                height: t.height,
                levels: t.mip_levels.len() as u32,
                gl_internal_format,
                compressed,
                bytes: t.byte_len(),
                mip_levels: &t.mip_levels,
                repeat: false,
            }
        })
        .collect::<Vec<_>>();

    let mut order: Vec<usize> = (0..visual.meshes.len()).collect();
    order.sort_by_key(|&i| kind_rank(visual.meshes[i].kind));
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut batches = Vec::with_capacity(visual.meshes.len());
    let mut triangles = [0u64; 4];
    for i in order {
        let mesh = &visual.meshes[i];
        let material = visual.materials.iter().find(|m| m.id == mesh.material).expect("validated material");
        let base = u32::try_from(vertices.len()).map_err(|_| "photo vertex count exceeds u32")?;
        let first = u32::try_from(indices.len()).map_err(|_| "photo index count exceeds u32")?;
        vertices.extend(mesh.vertices.iter().map(|v| TexturedVertex { position: v.position, normal: v.normal, uv: v.uv, animation: v.animation as f32 }));
        for &index in &mesh.indices {
            indices.push(base.checked_add(index).ok_or("photo index exceeds u32")?);
        }
        triangles[kind_rank(mesh.kind) as usize] += (mesh.indices.len() / 3) as u64;
        batches.push(PhotoBatch {
            kind: mesh.kind,
            material_id: material.id,
            texture_id: material.base_color_texture,
            first_index: first,
            index_count: mesh.indices.len() as u32,
            base_color_factor: material.base_color_factor,
            roughness: material.roughness,
            metallic: material.metallic,
        });
    }
    for t in &mut textures {
        let mut users = batches.iter().filter(|b| b.texture_id == t.id);
        t.repeat = users.clone().next().is_some() && users.all(|b| b.kind == VisualKind::Building);
    }
    let texture_bytes: u64 = textures.iter().map(|t| t.bytes).sum();
    let vertex_bytes = (vertices.len() * std::mem::size_of::<TexturedVertex>()) as u64;
    let index_bytes = (indices.len() * 4) as u64;
    let memory = PhotoMemory {
        texture_bytes,
        vertex_bytes,
        index_bytes,
        gpu_static_bytes: texture_bytes + vertex_bytes + index_bytes,
        triangles,
    };
    let joints = if visual.format_minor >= 2 { visual.joints.clone() } else { legacy_joints() };
    Ok(PhotoPlan { textures, vertices, indices, batches, memory, joints })
}
