use std::collections::HashSet;
use std::fmt;
use std::io::{Read, Write};

use crate::{
    Bounds3, Chunk, GeodeticPoint, GeoidModel, Georeference, Layer, Light, LightLayer, LightType, Lod, Mesh, NamedMesh,
    WorldAsset,
};

const MAGIC: &[u8; 8] = b"GXWORLD\0";
const HEADER_BYTES: usize = 40;
const FNV_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

/// Format history:
/// * 1.0 - origin + chunks. No frame identity or geoid separation; rejected
///   by this reader (`FormatError::ObsoleteVersion`), never migrated,
///   because the missing separation cannot be inferred safely.
/// * 2.0 - adds the tagged georeference block after the origin.
pub const FORMAT_MAJOR: u16 = 2;
pub const FORMAT_MINOR: u16 = 0;
/// Payload tag `GEOR` that opens the georeference block, followed by its
/// byte length and then: frame id u64, frame generation u64, geoid model
/// u32, reserved u32 (zero), geoid separation f64 metres.
const GEOREFERENCE_TAG: u32 = u32::from_le_bytes(*b"GEOR");
const GEOREFERENCE_BYTES: u32 = 32;
/// Header tag for WGS84 origin plus local x=east, y=north, z=up metres.
pub const COORDINATE_SYSTEM_WGS84_ENU_METRES: u32 = 1;

#[derive(Clone, Copy, Debug)]
pub struct Limits {
    pub max_file_bytes: u64,
    pub max_chunks: u32,
    pub max_lods_per_chunk: u16,
    pub max_layers_per_lod: u16,
    pub max_vertices_per_mesh: u32,
    pub max_indices_per_mesh: u32,
    pub max_lights_per_layer: u32,
    pub max_name_bytes: u16,
}

impl Default for Limits {
    fn default() -> Self {
        Self {
            max_file_bytes: 256 * 1024 * 1024,
            max_chunks: 16_384,
            max_lods_per_chunk: 8,
            max_layers_per_lod: 64,
            max_vertices_per_mesh: 2_000_000,
            max_indices_per_mesh: 6_000_000,
            max_lights_per_layer: 1_000_000,
            max_name_bytes: 128,
        }
    }
}

#[derive(Debug)]
pub enum FormatError {
    Io(std::io::Error),
    Invalid(&'static str),
    UnsupportedVersion { major: u16, minor: u16 },
    /// A known older version this reader refuses (no migration).
    ObsoleteVersion { major: u16, minor: u16 },
    LimitExceeded(&'static str),
    HashMismatch { expected: u64, actual: u64 },
    InvalidUtf8,
}

impl fmt::Display for FormatError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(f, "I/O error: {error}"),
            Self::Invalid(reason) => write!(f, "invalid world asset: {reason}"),
            Self::UnsupportedVersion { major, minor } => {
                write!(f, "unsupported world format {major}.{minor}")
            }
            Self::ObsoleteVersion { major, minor } => write!(
                f,
                "world format {major}.{minor} is obsolete: it lacks the frame id/generation                  and MSL-to-ellipsoid geoid separation required by format                  {FORMAT_MAJOR}.{FORMAT_MINOR}; regenerate the pack with the offline tool"
            ),
            Self::LimitExceeded(name) => write!(f, "world asset exceeds {name}"),
            Self::HashMismatch { expected, actual } => write!(
                f,
                "world payload hash mismatch: expected {expected:016x}, got {actual:016x}"
            ),
            Self::InvalidUtf8 => f.write_str("world asset contains invalid UTF-8"),
        }
    }
}

impl std::error::Error for FormatError {}

impl From<std::io::Error> for FormatError {
    fn from(value: std::io::Error) -> Self {
        Self::Io(value)
    }
}

pub fn write_world<W: Write>(
    world: &WorldAsset,
    limits: Limits,
    mut output: W,
) -> Result<(), FormatError> {
    validate_world(world, limits)?;
    let mut payload = Vec::new();
    put_f64(&mut payload, world.origin.latitude_deg);
    put_f64(&mut payload, world.origin.longitude_deg);
    put_f64(&mut payload, world.origin.height_m);
    let geo = world.georeference;
    put_u32(&mut payload, GEOREFERENCE_TAG);
    put_u32(&mut payload, GEOREFERENCE_BYTES);
    put_u64(&mut payload, geo.frame_id);
    put_u64(&mut payload, geo.frame_generation);
    put_u32(&mut payload, geo.geoid_model as u32);
    put_u32(&mut payload, 0);
    put_f64(&mut payload, geo.geoid_separation_m);
    for chunk in &world.chunks {
        encode_chunk(&mut payload, chunk);
    }

    let total = HEADER_BYTES
        .checked_add(payload.len())
        .ok_or(FormatError::LimitExceeded("max_file_bytes"))?;
    if total as u64 > limits.max_file_bytes {
        return Err(FormatError::LimitExceeded("max_file_bytes"));
    }

    let mut header = Vec::with_capacity(HEADER_BYTES);
    header.extend_from_slice(MAGIC);
    put_u16(&mut header, FORMAT_MAJOR);
    put_u16(&mut header, FORMAT_MINOR);
    put_u32(&mut header, HEADER_BYTES as u32);
    put_u64(&mut header, payload.len() as u64);
    put_u64(&mut header, payload_hash(&payload));
    put_u32(&mut header, world.chunks.len() as u32);
    put_u32(&mut header, COORDINATE_SYSTEM_WGS84_ENU_METRES);
    debug_assert_eq!(header.len(), HEADER_BYTES);
    output.write_all(&header)?;
    output.write_all(&payload)?;
    Ok(())
}

pub fn read_world<R: Read>(mut input: R, limits: Limits) -> Result<WorldAsset, FormatError> {
    let read_cap = limits
        .max_file_bytes
        .checked_add(1)
        .ok_or(FormatError::LimitExceeded("max_file_bytes"))?;
    let mut bytes = Vec::new();
    input.by_ref().take(read_cap).read_to_end(&mut bytes)?;
    if bytes.len() as u64 > limits.max_file_bytes {
        return Err(FormatError::LimitExceeded("max_file_bytes"));
    }
    if bytes.len() < HEADER_BYTES {
        return Err(FormatError::Invalid("truncated header"));
    }
    if &bytes[..8] != MAGIC {
        return Err(FormatError::Invalid("bad magic"));
    }
    let mut header = Cursor::new(&bytes[8..]);
    let major = header.u16()?;
    let minor = header.u16()?;
    if major == 1 {
        return Err(FormatError::ObsoleteVersion { major, minor });
    }
    if major != FORMAT_MAJOR || minor > FORMAT_MINOR {
        return Err(FormatError::UnsupportedVersion { major, minor });
    }
    if header.u32()? as usize != HEADER_BYTES {
        return Err(FormatError::Invalid("unexpected header size"));
    }
    let payload_len =
        usize::try_from(header.u64()?).map_err(|_| FormatError::LimitExceeded("max_file_bytes"))?;
    let expected_hash = header.u64()?;
    let chunk_count = header.u32()?;
    if header.u32()? != COORDINATE_SYSTEM_WGS84_ENU_METRES {
        return Err(FormatError::Invalid("unsupported coordinate system"));
    }
    if chunk_count > limits.max_chunks {
        return Err(FormatError::LimitExceeded("max_chunks"));
    }
    if payload_len != bytes.len() - HEADER_BYTES {
        return Err(FormatError::Invalid("payload length mismatch"));
    }
    let payload = &bytes[HEADER_BYTES..];
    let actual_hash = payload_hash(payload);
    if actual_hash != expected_hash {
        return Err(FormatError::HashMismatch {
            expected: expected_hash,
            actual: actual_hash,
        });
    }

    let mut cursor = Cursor::new(payload);
    let origin = GeodeticPoint {
        latitude_deg: cursor.f64()?,
        longitude_deg: cursor.f64()?,
        height_m: cursor.f64()?,
    };
    let georeference = decode_georeference(&mut cursor)?;
    let mut chunks = Vec::with_capacity(chunk_count as usize);
    for _ in 0..chunk_count {
        chunks.push(decode_chunk(&mut cursor, limits)?);
    }
    if cursor.remaining() != 0 {
        return Err(FormatError::Invalid("trailing payload bytes"));
    }
    let world = WorldAsset { origin, georeference, chunks };
    validate_world(&world, limits)?;
    Ok(world)
}

fn validate_world(world: &WorldAsset, limits: Limits) -> Result<(), FormatError> {
    if !world.origin.latitude_deg.is_finite()
        || !(-90.0..=90.0).contains(&world.origin.latitude_deg)
    {
        return Err(FormatError::Invalid("origin latitude"));
    }
    if !world.origin.longitude_deg.is_finite()
        || !(-180.0..=180.0).contains(&world.origin.longitude_deg)
    {
        return Err(FormatError::Invalid("origin longitude"));
    }
    if !world.origin.height_m.is_finite() {
        return Err(FormatError::Invalid("origin height"));
    }
    validate_georeference(world.georeference)?;
    if world.chunks.len() > limits.max_chunks as usize {
        return Err(FormatError::LimitExceeded("max_chunks"));
    }
    if world.chunks.is_empty() {
        return Err(FormatError::Invalid("world has no chunks"));
    }
    let mut ids = HashSet::with_capacity(world.chunks.len());
    for chunk in &world.chunks {
        if !ids.insert(chunk.id) {
            return Err(FormatError::Invalid("duplicate chunk id"));
        }
        validate_bounds(chunk.bounds)?;
        if chunk.lods.is_empty() {
            return Err(FormatError::Invalid("chunk has no LOD"));
        }
        if chunk.lods.len() > limits.max_lods_per_chunk as usize {
            return Err(FormatError::LimitExceeded("max_lods_per_chunk"));
        }
        let mut levels = HashSet::with_capacity(chunk.lods.len());
        let mut previous_level = None;
        let mut previous_max_distance = None;
        for lod in &chunk.lods {
            if !levels.insert(lod.level) {
                return Err(FormatError::Invalid("duplicate LOD level"));
            }
            if !lod.min_distance_m.is_finite()
                || !lod.max_distance_m.is_finite()
                || lod.min_distance_m < 0.0
                || lod.max_distance_m <= lod.min_distance_m
            {
                return Err(FormatError::Invalid("LOD distance range"));
            }
            if previous_level.is_some_and(|level| lod.level <= level) {
                return Err(FormatError::Invalid("LOD levels are not increasing"));
            }
            if previous_max_distance.is_some_and(|distance| lod.min_distance_m < distance) {
                return Err(FormatError::Invalid("LOD distance ranges overlap"));
            }
            previous_level = Some(lod.level);
            previous_max_distance = Some(lod.max_distance_m);
            if lod.layers.len() > limits.max_layers_per_lod as usize {
                return Err(FormatError::LimitExceeded("max_layers_per_lod"));
            }
            if lod.layers.is_empty() {
                return Err(FormatError::Invalid("LOD has no layers"));
            }
            for layer in &lod.layers {
                match layer {
                    Layer::Land(mesh) | Layer::Water(mesh) | Layer::Markings(mesh) => {
                        validate_mesh(mesh, chunk.bounds, limits)?
                    }
                    Layer::Runway(named) | Layer::Taxiway(named) => {
                        validate_name(&named.name, limits)?;
                        validate_mesh(&named.mesh, chunk.bounds, limits)?;
                    }
                    Layer::Lights(layer) => validate_lights(layer, chunk.bounds, limits)?,
                }
            }
        }
    }
    Ok(())
}

fn validate_georeference(geo: Georeference) -> Result<(), FormatError> {
    if geo.frame_id == 0 {
        return Err(FormatError::Invalid("frame id is zero"));
    }
    if geo.frame_generation == 0 {
        return Err(FormatError::Invalid("frame generation is zero"));
    }
    if !geo.geoid_separation_m.is_finite()
        || !Georeference::SEPARATION_RANGE_M.contains(&geo.geoid_separation_m)
    {
        return Err(FormatError::Invalid("geoid separation out of range"));
    }
    Ok(())
}

fn decode_georeference(cursor: &mut Cursor<'_>) -> Result<Georeference, FormatError> {
    if cursor.u32()? != GEOREFERENCE_TAG {
        return Err(FormatError::Invalid("missing georeference block"));
    }
    if cursor.u32()? != GEOREFERENCE_BYTES {
        return Err(FormatError::Invalid("georeference block size"));
    }
    let frame_id = cursor.u64()?;
    let frame_generation = cursor.u64()?;
    let geoid_model = GeoidModel::from_u32(cursor.u32()?)
        .ok_or(FormatError::Invalid("unknown geoid model"))?;
    if cursor.u32()? != 0 {
        return Err(FormatError::Invalid("georeference reserved field"));
    }
    let geo = Georeference {
        frame_id,
        frame_generation,
        geoid_model,
        geoid_separation_m: cursor.f64()?,
    };
    validate_georeference(geo)?;
    Ok(geo)
}

fn validate_bounds(bounds: Bounds3) -> Result<(), FormatError> {
    for axis in 0..3 {
        if !bounds.min[axis].is_finite()
            || !bounds.max[axis].is_finite()
            || bounds.min[axis] > bounds.max[axis]
        {
            return Err(FormatError::Invalid("chunk bounds"));
        }
    }
    Ok(())
}

fn validate_name(name: &str, limits: Limits) -> Result<(), FormatError> {
    if name.is_empty() {
        return Err(FormatError::Invalid("empty layer name"));
    }
    if name.len() > limits.max_name_bytes as usize {
        return Err(FormatError::LimitExceeded("max_name_bytes"));
    }
    Ok(())
}

fn validate_mesh(mesh: &Mesh, bounds: Bounds3, limits: Limits) -> Result<(), FormatError> {
    if mesh.positions.len() > limits.max_vertices_per_mesh as usize {
        return Err(FormatError::LimitExceeded("max_vertices_per_mesh"));
    }
    if mesh.indices.len() > limits.max_indices_per_mesh as usize {
        return Err(FormatError::LimitExceeded("max_indices_per_mesh"));
    }
    if mesh.positions.is_empty()
        || mesh.indices.is_empty()
        || !mesh.indices.chunks_exact(3).remainder().is_empty()
    {
        return Err(FormatError::Invalid("mesh topology"));
    }
    for position in &mesh.positions {
        validate_position(*position, bounds)?;
    }
    if mesh
        .indices
        .iter()
        .any(|&index| index as usize >= mesh.positions.len())
    {
        return Err(FormatError::Invalid("mesh index out of range"));
    }
    Ok(())
}

fn validate_lights(layer: &LightLayer, bounds: Bounds3, limits: Limits) -> Result<(), FormatError> {
    if layer.lights.len() > limits.max_lights_per_layer as usize {
        return Err(FormatError::LimitExceeded("max_lights_per_layer"));
    }
    if layer.lights.is_empty() {
        return Err(FormatError::Invalid("light layer is empty"));
    }
    for light in &layer.lights {
        validate_position(light.position, bounds)?;
        if !light.intensity_cd.is_finite() || light.intensity_cd < 0.0 {
            return Err(FormatError::Invalid("light intensity"));
        }
    }
    Ok(())
}

fn validate_position(position: [f32; 3], bounds: Bounds3) -> Result<(), FormatError> {
    for (axis, value) in position.into_iter().enumerate() {
        if !value.is_finite() || value < bounds.min[axis] || value > bounds.max[axis] {
            return Err(FormatError::Invalid("position outside chunk bounds"));
        }
    }
    Ok(())
}

fn encode_chunk(output: &mut Vec<u8>, chunk: &Chunk) {
    put_u32(output, chunk.id);
    for value in chunk.bounds.min.into_iter().chain(chunk.bounds.max) {
        put_f32(output, value);
    }
    put_u16(output, chunk.lods.len() as u16);
    for lod in &chunk.lods {
        output.push(lod.level);
        put_f32(output, lod.min_distance_m);
        put_f32(output, lod.max_distance_m);
        put_u16(output, lod.layers.len() as u16);
        for layer in &lod.layers {
            output.push(layer.tag());
            match layer {
                Layer::Land(mesh) | Layer::Water(mesh) | Layer::Markings(mesh) => {
                    encode_mesh(output, mesh)
                }
                Layer::Runway(named) | Layer::Taxiway(named) => {
                    put_u16(output, named.name.len() as u16);
                    output.extend_from_slice(named.name.as_bytes());
                    encode_mesh(output, &named.mesh);
                }
                Layer::Lights(layer) => {
                    put_u32(output, layer.lights.len() as u32);
                    for light in &layer.lights {
                        for value in light.position {
                            put_f32(output, value);
                        }
                        output.extend_from_slice(&light.color_rgb8);
                        output.push(light.kind as u8);
                        put_f32(output, light.intensity_cd);
                    }
                }
            }
        }
    }
}

fn encode_mesh(output: &mut Vec<u8>, mesh: &Mesh) {
    put_u32(output, mesh.positions.len() as u32);
    put_u32(output, mesh.indices.len() as u32);
    for position in &mesh.positions {
        for value in position {
            put_f32(output, *value);
        }
    }
    for &index in &mesh.indices {
        put_u32(output, index);
    }
}

fn decode_chunk(cursor: &mut Cursor<'_>, limits: Limits) -> Result<Chunk, FormatError> {
    let id = cursor.u32()?;
    let mut values = [0.0; 6];
    for value in &mut values {
        *value = cursor.f32()?;
    }
    let bounds = Bounds3 {
        min: [values[0], values[1], values[2]],
        max: [values[3], values[4], values[5]],
    };
    let lod_count = cursor.u16()?;
    if lod_count > limits.max_lods_per_chunk {
        return Err(FormatError::LimitExceeded("max_lods_per_chunk"));
    }
    let mut lods = Vec::with_capacity(lod_count as usize);
    for _ in 0..lod_count {
        let level = cursor.u8()?;
        let min_distance_m = cursor.f32()?;
        let max_distance_m = cursor.f32()?;
        let layer_count = cursor.u16()?;
        if layer_count > limits.max_layers_per_lod {
            return Err(FormatError::LimitExceeded("max_layers_per_lod"));
        }
        let mut layers = Vec::with_capacity(layer_count as usize);
        for _ in 0..layer_count {
            layers.push(decode_layer(cursor, limits)?);
        }
        lods.push(Lod {
            level,
            min_distance_m,
            max_distance_m,
            layers,
        });
    }
    Ok(Chunk { id, bounds, lods })
}

fn decode_layer(cursor: &mut Cursor<'_>, limits: Limits) -> Result<Layer, FormatError> {
    Ok(match cursor.u8()? {
        1 => Layer::Land(decode_mesh(cursor, limits)?),
        2 => Layer::Water(decode_mesh(cursor, limits)?),
        3 => Layer::Runway(decode_named_mesh(cursor, limits)?),
        4 => Layer::Taxiway(decode_named_mesh(cursor, limits)?),
        5 => Layer::Markings(decode_mesh(cursor, limits)?),
        6 => {
            let count = cursor.u32()?;
            if count > limits.max_lights_per_layer {
                return Err(FormatError::LimitExceeded("max_lights_per_layer"));
            }
            cursor.require(
                (count as usize)
                    .checked_mul(20)
                    .ok_or(FormatError::LimitExceeded("max_lights_per_layer"))?,
            )?;
            let mut lights = Vec::with_capacity(count as usize);
            for _ in 0..count {
                let position = [cursor.f32()?, cursor.f32()?, cursor.f32()?];
                let color_rgb8 = [cursor.u8()?, cursor.u8()?, cursor.u8()?];
                let kind = LightType::from_byte(cursor.u8()?)
                    .ok_or(FormatError::Invalid("unknown light type"))?;
                let intensity_cd = cursor.f32()?;
                lights.push(Light {
                    position,
                    color_rgb8,
                    kind,
                    intensity_cd,
                });
            }
            Layer::Lights(LightLayer { lights })
        }
        _ => return Err(FormatError::Invalid("unknown layer type")),
    })
}

fn decode_named_mesh(cursor: &mut Cursor<'_>, limits: Limits) -> Result<NamedMesh, FormatError> {
    let name_len = cursor.u16()?;
    if name_len > limits.max_name_bytes {
        return Err(FormatError::LimitExceeded("max_name_bytes"));
    }
    let name = std::str::from_utf8(cursor.bytes(name_len as usize)?)
        .map_err(|_| FormatError::InvalidUtf8)?
        .to_owned();
    Ok(NamedMesh {
        name,
        mesh: decode_mesh(cursor, limits)?,
    })
}

fn decode_mesh(cursor: &mut Cursor<'_>, limits: Limits) -> Result<Mesh, FormatError> {
    let vertex_count = cursor.u32()?;
    let index_count = cursor.u32()?;
    if vertex_count > limits.max_vertices_per_mesh {
        return Err(FormatError::LimitExceeded("max_vertices_per_mesh"));
    }
    if index_count > limits.max_indices_per_mesh {
        return Err(FormatError::LimitExceeded("max_indices_per_mesh"));
    }
    let position_bytes = (vertex_count as usize)
        .checked_mul(12)
        .ok_or(FormatError::LimitExceeded("max_vertices_per_mesh"))?;
    let index_bytes = (index_count as usize)
        .checked_mul(4)
        .ok_or(FormatError::LimitExceeded("max_indices_per_mesh"))?;
    cursor.require(
        position_bytes
            .checked_add(index_bytes)
            .ok_or(FormatError::LimitExceeded("max_file_bytes"))?,
    )?;
    let mut positions = Vec::with_capacity(vertex_count as usize);
    for _ in 0..vertex_count {
        positions.push([cursor.f32()?, cursor.f32()?, cursor.f32()?]);
    }
    let mut indices = Vec::with_capacity(index_count as usize);
    for _ in 0..index_count {
        indices.push(cursor.u32()?);
    }
    Ok(Mesh { positions, indices })
}

fn payload_hash(bytes: &[u8]) -> u64 {
    let mut hash = FNV_OFFSET;
    for &byte in bytes {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(FNV_PRIME);
    }
    hash
}

fn put_u16(output: &mut Vec<u8>, value: u16) {
    output.extend_from_slice(&value.to_le_bytes());
}
fn put_u32(output: &mut Vec<u8>, value: u32) {
    output.extend_from_slice(&value.to_le_bytes());
}
fn put_u64(output: &mut Vec<u8>, value: u64) {
    output.extend_from_slice(&value.to_le_bytes());
}
fn put_f32(output: &mut Vec<u8>, value: f32) {
    output.extend_from_slice(&value.to_le_bytes());
}
fn put_f64(output: &mut Vec<u8>, value: f64) {
    output.extend_from_slice(&value.to_le_bytes());
}

struct Cursor<'a> {
    bytes: &'a [u8],
    offset: usize,
}

impl<'a> Cursor<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, offset: 0 }
    }
    fn remaining(&self) -> usize {
        self.bytes.len() - self.offset
    }
    fn require(&self, count: usize) -> Result<(), FormatError> {
        if count > self.remaining() {
            Err(FormatError::Invalid("truncated payload"))
        } else {
            Ok(())
        }
    }
    fn bytes(&mut self, count: usize) -> Result<&'a [u8], FormatError> {
        self.require(count)?;
        let start = self.offset;
        self.offset += count;
        Ok(&self.bytes[start..self.offset])
    }
    fn u8(&mut self) -> Result<u8, FormatError> {
        Ok(self.bytes(1)?[0])
    }
    fn u16(&mut self) -> Result<u16, FormatError> {
        Ok(u16::from_le_bytes(self.bytes(2)?.try_into().unwrap()))
    }
    fn u32(&mut self) -> Result<u32, FormatError> {
        Ok(u32::from_le_bytes(self.bytes(4)?.try_into().unwrap()))
    }
    fn u64(&mut self) -> Result<u64, FormatError> {
        Ok(u64::from_le_bytes(self.bytes(8)?.try_into().unwrap()))
    }
    fn f32(&mut self) -> Result<f32, FormatError> {
        Ok(f32::from_le_bytes(self.bytes(4)?.try_into().unwrap()))
    }
    fn f64(&mut self) -> Result<f64, FormatError> {
        Ok(f64::from_le_bytes(self.bytes(8)?.try_into().unwrap()))
    }
}
