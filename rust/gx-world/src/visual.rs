//! Versioned photoreal visual sidecar.
//!
//! `GXWORLD` remains authoritative for georeference, collision/surface
//! queries and the S4 fallback.  This sidecar is optional and carries only
//! GPU-facing textured terrain and aircraft data.  Both packs are joined by
//! the exact `(frame_id, frame_generation)` pair, so stale imagery can never
//! silently move relative to the runway.

use std::collections::HashSet;
use std::fmt;
use std::io::{Read, Write};

pub const VISUAL_FORMAT_MAJOR: u16 = 1;
pub const VISUAL_FORMAT_MINOR: u16 = 2;
/// GXVISUAL 1.2 joint-table bounds: joints (the renderer uploads one 3x4
/// matrix per joint plus the static identity) and points per table.
pub const MAX_JOINTS: usize = 64;
pub const MAX_JOINT_POINTS: usize = 8;
/// Building UVs may repeat a texture; |uv| is bounded to keep f32 texel
/// precision (4096 tiles of 256 px is 2^20 texels).
pub const MAX_BUILDING_UV: f32 = 4096.0;

/// What moves a joint. Inputs are the FDM's normalized positions:
/// aileron/elevator/rudder -1..1 (JSBSim `*-pos-norm`, left aileron for the
/// aileron), flap/speedbrake/gear 0..1.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointDriver {
    Aileron = 1,
    Elevator = 2,
    Rudder = 3,
    Flap = 4,
    Speedbrake = 5,
    Gear = 6,
}

impl JointDriver {
    pub const ALL: [JointDriver; 6] = [
        Self::Aileron, Self::Elevator, Self::Rudder, Self::Flap, Self::Speedbrake, Self::Gear,
    ];

    pub fn from_code(code: u8) -> Option<Self> {
        Self::ALL.into_iter().find(|d| *d as u8 == code)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    /// Rotate about `axis` through `center`; table outputs are degrees,
    /// right-handed about the axis.
    Rotate = 1,
    /// Translate along `axis`; table outputs are metres.
    Translate = 2,
}

/// One rigid aircraft joint (GXVISUAL 1.2), in body FRD metres about the
/// CG. A vertex whose `animation` is joint `j` (1-based) is transformed by
/// `world(j) = world(parent) * local(j)`, so the outermost joint of a chain
/// is the root. The driver value is mapped through `table`, a piecewise
/// linear function with strictly ascending inputs that is held constant
/// outside its range.
#[derive(Clone, Debug, PartialEq)]
pub struct Joint {
    /// 0 = none, else a 1-based joint id smaller than this joint's own.
    pub parent: u32,
    pub driver: JointDriver,
    pub kind: JointKind,
    pub center: [f32; 3],
    /// Unit vector.
    pub axis: [f32; 3],
    /// `(input, output)` points, 2..=[`MAX_JOINT_POINTS`].
    pub table: Vec<[f32; 2]>,
}

impl Joint {
    /// The table's output at `input` (held at the ends; non-finite input
    /// reads as 0).
    pub fn evaluate(&self, input: f32) -> f32 {
        let x = if input.is_finite() { input } else { 0.0 };
        let t = &self.table;
        if x <= t[0][0] {
            return t[0][1];
        }
        for w in t.windows(2) {
            if x <= w[1][0] {
                let f = (x - w[0][0]) / (w[1][0] - w[0][0]);
                return w[0][1] + f * (w[1][1] - w[0][1]);
            }
        }
        t[t.len() - 1][1]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TextureEncoding {
    /// Linear ASTC 6x6 blocks; colour is decoded as sRGB by the shader.
    Astc6x6Srgb,
    /// Portable development/fallback payload, tightly packed RGBA8.
    Rgba8Srgb,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Texture {
    pub id: u32,
    pub width: u32,
    pub height: u32,
    /// Complete mip chain, largest level first. Each level is tightly packed.
    pub mip_levels: Vec<Vec<u8>>,
    pub encoding: TextureEncoding,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Material {
    pub id: u32,
    pub base_color_texture: u32,
    pub base_color_factor: [f32; 4],
    pub roughness: f32,
    pub metallic: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VisualKind {
    /// Depth-tested ground in absolute ENU metres (texture alpha < 1 is water).
    Terrain,
    /// Body-frame (FRD, metres about the JSBSim CG) aircraft geometry.
    Aircraft,
    /// Flat surface paint (runway/taxiway markings) in absolute ENU metres,
    /// drawn over the terrain in painter's order without a depth test.
    Decal,
    /// GXVISUAL 1.2: 3D structures (airport buildings) in absolute ENU
    /// metres, depth tested and written after the decals, so markings never
    /// paint over a wall and buildings hide what is behind them. A texture
    /// that only Building meshes use is sampled with repeat (facade tiles
    /// in metre UVs); all others clamp to edge.
    Building,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct VisualVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub uv: [f32; 2],
    /// Rigid aircraft animation channel. Zero is static. In 1.1, values
    /// 1..=5 are the fixed bomb-5000 hinges (right/left aileron, right/left
    /// elevator, rudder); in 1.2 it is a 1-based id into
    /// [`VisualAsset::joints`]. GXVISUAL 1.0 assets decode as zero.
    pub animation: u32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct VisualMesh {
    pub kind: VisualKind,
    pub material: u32,
    pub vertices: Vec<VisualVertex>,
    pub indices: Vec<u32>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct VisualAsset {
    pub format_major: u16,
    pub format_minor: u16,
    pub frame_id: u64,
    pub frame_generation: u64,
    pub textures: Vec<Texture>,
    pub materials: Vec<Material>,
    pub meshes: Vec<VisualMesh>,
    /// GXVISUAL 1.2 joint table; empty before 1.2.
    pub joints: Vec<Joint>,
}

#[derive(Clone, Copy, Debug)]
pub struct VisualLimits {
    pub max_texture_bytes: u64,
    pub max_textures: usize,
    pub max_materials: usize,
    pub max_meshes: usize,
    pub max_vertices: usize,
    pub max_indices: usize,
}

impl Default for VisualLimits {
    fn default() -> Self {
        Self {
            // Deliberately below the board's ~1 GiB shared-memory ceiling.
            max_texture_bytes: 768 * 1024 * 1024,
            max_textures: 4096,
            max_materials: 4096,
            max_meshes: 16_384,
            max_vertices: 16_000_000,
            max_indices: 48_000_000,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum VisualError {
    UnsupportedVersion { major: u16, minor: u16 },
    LimitExceeded(&'static str),
    Invalid(&'static str),
}

impl fmt::Display for VisualError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedVersion { major, minor } => {
                write!(f, "unsupported visual format {major}.{minor}")
            }
            Self::LimitExceeded(name) => write!(f, "visual asset exceeds {name}"),
            Self::Invalid(reason) => write!(f, "invalid visual asset: {reason}"),
        }
    }
}

impl std::error::Error for VisualError {}

impl Texture {
    pub fn byte_len(&self) -> u64 {
        self.mip_levels.iter().map(|m| m.len() as u64).sum()
    }
}

impl VisualAsset {
    /// Validate untrusted offline-tool output before it reaches GLES.
    pub fn validate(&self, limits: VisualLimits) -> Result<(), VisualError> {
        if self.format_major != VISUAL_FORMAT_MAJOR || self.format_minor > VISUAL_FORMAT_MINOR {
            return Err(VisualError::UnsupportedVersion {
                major: self.format_major,
                minor: self.format_minor,
            });
        }
        if self.frame_id == 0 || self.frame_generation == 0 {
            return Err(VisualError::Invalid("zero frame identity"));
        }
        if self.textures.is_empty() || self.materials.is_empty() || self.meshes.is_empty() {
            return Err(VisualError::Invalid("empty visual collection"));
        }
        if self.textures.len() > limits.max_textures {
            return Err(VisualError::LimitExceeded("max_textures"));
        }
        if self.materials.len() > limits.max_materials {
            return Err(VisualError::LimitExceeded("max_materials"));
        }
        if self.meshes.len() > limits.max_meshes {
            return Err(VisualError::LimitExceeded("max_meshes"));
        }

        let mut texture_ids = HashSet::new();
        let mut texture_bytes = 0u64;
        for texture in &self.textures {
            if texture.id == 0 || !texture_ids.insert(texture.id) {
                return Err(VisualError::Invalid("duplicate or zero texture id"));
            }
            if texture.width == 0 || texture.height == 0 || texture.mip_levels.is_empty() {
                return Err(VisualError::Invalid("texture dimensions or mip chain"));
            }
            let expected_levels = 32 - texture.width.max(texture.height).leading_zeros();
            if texture.mip_levels.len() > expected_levels as usize {
                return Err(VisualError::Invalid("too many texture mip levels"));
            }
            let mut w = texture.width;
            let mut h = texture.height;
            for mip in &texture.mip_levels {
                let expected = match texture.encoding {
                    TextureEncoding::Rgba8Srgb => u64::from(w) * u64::from(h) * 4,
                    TextureEncoding::Astc6x6Srgb => {
                        u64::from(w.div_ceil(6)) * u64::from(h.div_ceil(6)) * 16
                    }
                };
                if mip.len() as u64 != expected {
                    return Err(VisualError::Invalid("texture mip byte length"));
                }
                w = (w / 2).max(1);
                h = (h / 2).max(1);
            }
            texture_bytes = texture_bytes
                .checked_add(texture.byte_len())
                .ok_or(VisualError::LimitExceeded("max_texture_bytes"))?;
        }
        if texture_bytes > limits.max_texture_bytes {
            return Err(VisualError::LimitExceeded("max_texture_bytes"));
        }

        let mut material_ids = HashSet::new();
        for material in &self.materials {
            if material.id == 0 || !material_ids.insert(material.id) {
                return Err(VisualError::Invalid("duplicate or zero material id"));
            }
            if !texture_ids.contains(&material.base_color_texture) {
                return Err(VisualError::Invalid("material texture reference"));
            }
            if !material
                .base_color_factor
                .iter()
                .all(|v| v.is_finite() && (0.0..=1.0).contains(v))
                || !material.roughness.is_finite()
                || !(0.0..=1.0).contains(&material.roughness)
                || !material.metallic.is_finite()
                || !(0.0..=1.0).contains(&material.metallic)
            {
                return Err(VisualError::Invalid("material values"));
            }
        }

        if self.format_minor < 2 && !self.joints.is_empty() {
            return Err(VisualError::Invalid("joints require GXVISUAL 1.2"));
        }
        if self.joints.len() > MAX_JOINTS {
            return Err(VisualError::LimitExceeded("max_joints"));
        }
        for (i, j) in self.joints.iter().enumerate() {
            let id = i as u32 + 1;
            let unit = j.axis.iter().map(|v| v * v).sum::<f32>().sqrt();
            let bound = match j.kind {
                JointKind::Rotate => 360.0,
                JointKind::Translate => 20.0,
            };
            if j.parent >= id
                || !j.center.iter().chain(j.axis.iter()).all(|v| v.is_finite())
                || !(0.999..=1.001).contains(&unit)
                || !(2..=MAX_JOINT_POINTS).contains(&j.table.len())
                || !j.table.iter().all(|p| p[0].is_finite() && p[1].is_finite() && p[1].abs() <= bound)
                || !j.table.windows(2).all(|w| w[1][0] > w[0][0])
            {
                return Err(VisualError::Invalid("joint values"));
            }
        }
        let max_animation = if self.format_minor >= 2 { self.joints.len() } else { 5 };

        let mut vertices = 0usize;
        let mut indices = 0usize;
        for mesh in &self.meshes {
            if !material_ids.contains(&mesh.material) {
                return Err(VisualError::Invalid("mesh material reference"));
            }
            if mesh.vertices.is_empty() || mesh.indices.is_empty() || mesh.indices.len() % 3 != 0 {
                return Err(VisualError::Invalid("mesh topology"));
            }
            if mesh
                .indices
                .iter()
                .any(|&i| i as usize >= mesh.vertices.len())
            {
                return Err(VisualError::Invalid("mesh index out of range"));
            }
            // Building UVs count repeats of a tiling facade texture.
            let uv_range = if mesh.kind == VisualKind::Building { -MAX_BUILDING_UV..=MAX_BUILDING_UV } else { 0.0..=1.0 };
            if mesh.vertices.iter().any(|v| {
                !v.position
                    .iter()
                    .chain(v.normal.iter())
                    .chain(v.uv.iter())
                    .all(|x| x.is_finite())
                    || v.uv.iter().any(|&x| !uv_range.contains(&x))
                    || v.animation as usize > max_animation
            }) {
                return Err(VisualError::Invalid("vertex values"));
            }
            if self.format_minor == 0 && mesh.vertices.iter().any(|v| v.animation != 0) {
                return Err(VisualError::Invalid("animation requires GXVISUAL 1.1"));
            }
            if mesh.kind == VisualKind::Building && self.format_minor < 2 {
                return Err(VisualError::Invalid("building meshes require GXVISUAL 1.2"));
            }
            if mesh.kind != VisualKind::Aircraft && mesh.vertices.iter().any(|v| v.animation != 0) {
                return Err(VisualError::Invalid("animation on non-aircraft mesh"));
            }
            vertices = vertices
                .checked_add(mesh.vertices.len())
                .ok_or(VisualError::LimitExceeded("max_vertices"))?;
            indices = indices
                .checked_add(mesh.indices.len())
                .ok_or(VisualError::LimitExceeded("max_indices"))?;
        }
        if vertices > limits.max_vertices {
            return Err(VisualError::LimitExceeded("max_vertices"));
        }
        if indices > limits.max_indices {
            return Err(VisualError::LimitExceeded("max_indices"));
        }
        Ok(())
    }

    pub fn matches_world(&self, world: &crate::WorldAsset) -> bool {
        self.frame_id == world.georeference.frame_id
            && self.frame_generation == world.georeference.frame_generation
    }
}

const VISUAL_MAGIC: &[u8; 8] = b"GXVISUAL";

/// Write the stable little-endian GXVISUAL 1.x interchange format used by
/// offline converters and the board runtime.
pub fn write_visual(
    mut out: impl Write,
    asset: &VisualAsset,
    limits: VisualLimits,
) -> Result<(), String> {
    asset.validate(limits).map_err(|e| e.to_string())?;
    out.write_all(VISUAL_MAGIC).map_err(|e| e.to_string())?;
    for v in [asset.format_major, asset.format_minor] {
        out.write_all(&v.to_le_bytes()).map_err(|e| e.to_string())?;
    }
    for v in [asset.frame_id, asset.frame_generation] {
        out.write_all(&v.to_le_bytes()).map_err(|e| e.to_string())?;
    }
    for n in [
        asset.textures.len(),
        asset.materials.len(),
        asset.meshes.len(),
    ] {
        out.write_all(
            &u32::try_from(n)
                .map_err(|_| "GXVISUAL collection exceeds u32")?
                .to_le_bytes(),
        )
        .map_err(|e| e.to_string())?;
    }
    for t in &asset.textures {
        for v in [t.id, t.width, t.height] {
            out.write_all(&v.to_le_bytes()).map_err(|e| e.to_string())?;
        }
        out.write_all(&[
            match t.encoding {
                TextureEncoding::Astc6x6Srgb => 1,
                TextureEncoding::Rgba8Srgb => 2,
            },
            0,
            0,
            0,
        ])
        .map_err(|e| e.to_string())?;
        out.write_all(&(t.mip_levels.len() as u32).to_le_bytes())
            .map_err(|e| e.to_string())?;
        for level in &t.mip_levels {
            out.write_all(&(level.len() as u64).to_le_bytes())
                .map_err(|e| e.to_string())?;
            out.write_all(level).map_err(|e| e.to_string())?;
        }
    }
    for m in &asset.materials {
        for v in [m.id, m.base_color_texture] {
            out.write_all(&v.to_le_bytes()).map_err(|e| e.to_string())?;
        }
        for v in m
            .base_color_factor
            .into_iter()
            .chain([m.roughness, m.metallic])
        {
            out.write_all(&v.to_le_bytes()).map_err(|e| e.to_string())?;
        }
    }
    for m in &asset.meshes {
        out.write_all(&[
            match m.kind {
                VisualKind::Terrain => 1,
                VisualKind::Aircraft => 2,
                VisualKind::Decal => 3,
                VisualKind::Building => 4,
            },
            0,
            0,
            0,
        ])
        .map_err(|e| e.to_string())?;
        for v in [m.material, m.vertices.len() as u32, m.indices.len() as u32] {
            out.write_all(&v.to_le_bytes()).map_err(|e| e.to_string())?;
        }
        for v in &m.vertices {
            for x in v.position.into_iter().chain(v.normal).chain(v.uv) {
                out.write_all(&x.to_le_bytes()).map_err(|e| e.to_string())?;
            }
            if asset.format_minor >= 1 {
                out.write_all(&v.animation.to_le_bytes()).map_err(|e| e.to_string())?;
            }
        }
        for i in &m.indices {
            out.write_all(&i.to_le_bytes()).map_err(|e| e.to_string())?;
        }
    }
    if asset.format_minor >= 2 {
        // Joint table: u32 count, then per joint u32 parent, u8 driver,
        // u8 kind, u8 point count, u8 0, f32 centre[3], f32 axis[3],
        // count x (f32 input, f32 output).
        out.write_all(&(asset.joints.len() as u32).to_le_bytes()).map_err(|e| e.to_string())?;
        for j in &asset.joints {
            out.write_all(&j.parent.to_le_bytes()).map_err(|e| e.to_string())?;
            out.write_all(&[j.driver as u8, j.kind as u8, j.table.len() as u8, 0]).map_err(|e| e.to_string())?;
            for v in j.center.into_iter().chain(j.axis).chain(j.table.iter().flatten().copied()) {
                out.write_all(&v.to_le_bytes()).map_err(|e| e.to_string())?;
            }
        }
    }
    Ok(())
}

/// Read a bounded GXVISUAL pack. Validation happens before the result is
/// returned; malformed converter output never reaches GLES.
pub fn read_visual(mut input: impl Read, limits: VisualLimits) -> Result<VisualAsset, String> {
    fn bytes<const N: usize>(r: &mut impl Read) -> Result<[u8; N], String> {
        let mut b = [0; N];
        r.read_exact(&mut b).map_err(|e| e.to_string())?;
        Ok(b)
    }
    fn u16v(r: &mut impl Read) -> Result<u16, String> {
        Ok(u16::from_le_bytes(bytes(r)?))
    }
    fn u32v(r: &mut impl Read) -> Result<u32, String> {
        Ok(u32::from_le_bytes(bytes(r)?))
    }
    fn u64v(r: &mut impl Read) -> Result<u64, String> {
        Ok(u64::from_le_bytes(bytes(r)?))
    }
    fn f32v(r: &mut impl Read) -> Result<f32, String> {
        Ok(f32::from_le_bytes(bytes(r)?))
    }
    if &bytes::<8>(&mut input)? != VISUAL_MAGIC {
        return Err("bad GXVISUAL magic".into());
    }
    let (format_major, format_minor) = (u16v(&mut input)?, u16v(&mut input)?);
    let (frame_id, frame_generation) = (u64v(&mut input)?, u64v(&mut input)?);
    let (nt, nm, ng) = (
        u32v(&mut input)? as usize,
        u32v(&mut input)? as usize,
        u32v(&mut input)? as usize,
    );
    if nt > limits.max_textures || nm > limits.max_materials || ng > limits.max_meshes {
        return Err("GXVISUAL collection limit exceeded".into());
    }
    let mut textures = Vec::with_capacity(nt);
    let mut texture_bytes = 0u64;
    for _ in 0..nt {
        let (id, width, height) = (u32v(&mut input)?, u32v(&mut input)?, u32v(&mut input)?);
        let enc = bytes::<4>(&mut input)?[0];
        let encoding = match enc {
            1 => TextureEncoding::Astc6x6Srgb,
            2 => TextureEncoding::Rgba8Srgb,
            _ => return Err("bad GXVISUAL texture encoding".into()),
        };
        let levels = u32v(&mut input)? as usize;
        if levels == 0 || levels > 32 {
            return Err("bad GXVISUAL mip count".into());
        }
        let mut mip_levels = Vec::with_capacity(levels);
        for _ in 0..levels {
            let len = u64v(&mut input)?;
            texture_bytes = texture_bytes
                .checked_add(len)
                .ok_or("GXVISUAL texture byte overflow")?;
            if texture_bytes > limits.max_texture_bytes || len > usize::MAX as u64 {
                return Err("GXVISUAL texture byte limit exceeded".into());
            }
            let mut data = vec![0; len as usize];
            input.read_exact(&mut data).map_err(|e| e.to_string())?;
            mip_levels.push(data);
        }
        textures.push(Texture {
            id,
            width,
            height,
            mip_levels,
            encoding,
        });
    }
    let mut materials = Vec::with_capacity(nm);
    for _ in 0..nm {
        let (id, base_color_texture) = (u32v(&mut input)?, u32v(&mut input)?);
        let mut base_color_factor = [0.0; 4];
        for x in &mut base_color_factor {
            *x = f32v(&mut input)?;
        }
        materials.push(Material {
            id,
            base_color_texture,
            base_color_factor,
            roughness: f32v(&mut input)?,
            metallic: f32v(&mut input)?,
        });
    }
    let mut meshes = Vec::with_capacity(ng);
    let (mut total_v, mut total_i) = (0usize, 0usize);
    for _ in 0..ng {
        let kind = match bytes::<4>(&mut input)?[0] {
            1 => VisualKind::Terrain,
            2 => VisualKind::Aircraft,
            3 => VisualKind::Decal,
            4 => VisualKind::Building,
            _ => return Err("bad GXVISUAL mesh kind".into()),
        };
        let material = u32v(&mut input)?;
        let nv = u32v(&mut input)? as usize;
        let ni = u32v(&mut input)? as usize;
        total_v = total_v
            .checked_add(nv)
            .ok_or("GXVISUAL vertex count overflow")?;
        total_i = total_i
            .checked_add(ni)
            .ok_or("GXVISUAL index count overflow")?;
        if total_v > limits.max_vertices || total_i > limits.max_indices {
            return Err("GXVISUAL geometry limit exceeded".into());
        }
        let mut vertices = Vec::with_capacity(nv);
        for _ in 0..nv {
            let mut position = [0.0; 3];
            let mut normal = [0.0; 3];
            let mut uv = [0.0; 2];
            for x in &mut position {
                *x = f32v(&mut input)?
            }
            for x in &mut normal {
                *x = f32v(&mut input)?
            }
            for x in &mut uv {
                *x = f32v(&mut input)?
            }
            vertices.push(VisualVertex {
                position,
                normal,
                uv,
                animation: if format_minor >= 1 { u32v(&mut input)? } else { 0 },
            });
        }
        let mut indices = Vec::with_capacity(ni);
        for _ in 0..ni {
            indices.push(u32v(&mut input)?);
        }
        meshes.push(VisualMesh {
            kind,
            material,
            vertices,
            indices,
        });
    }
    let mut joints = Vec::new();
    if format_minor >= 2 {
        let n = u32v(&mut input)? as usize;
        if n > MAX_JOINTS {
            return Err("GXVISUAL joint limit exceeded".into());
        }
        for _ in 0..n {
            let parent = u32v(&mut input)?;
            let [driver, kind, points, _] = bytes::<4>(&mut input)?;
            let driver = JointDriver::from_code(driver).ok_or("bad GXVISUAL joint driver")?;
            let kind = match kind {
                1 => JointKind::Rotate,
                2 => JointKind::Translate,
                _ => return Err("bad GXVISUAL joint kind".into()),
            };
            if !(2..=MAX_JOINT_POINTS).contains(&(points as usize)) {
                return Err("bad GXVISUAL joint table size".into());
            }
            let mut f = [0.0f32; 6];
            for x in &mut f {
                *x = f32v(&mut input)?;
            }
            let mut table = Vec::with_capacity(points as usize);
            for _ in 0..points {
                table.push([f32v(&mut input)?, f32v(&mut input)?]);
            }
            joints.push(Joint {
                parent,
                driver,
                kind,
                center: [f[0], f[1], f[2]],
                axis: [f[3], f[4], f[5]],
                table,
            });
        }
    }
    let asset = VisualAsset {
        format_major,
        format_minor,
        frame_id,
        frame_generation,
        textures,
        materials,
        meshes,
        joints,
    };
    asset.validate(limits).map_err(|e| e.to_string())?;
    Ok(asset)
}
