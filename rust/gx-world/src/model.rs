use crate::GeodeticPoint;

#[derive(Clone, Debug, PartialEq)]
pub struct WorldAsset {
    /// Local ENU origin. `height_m` is WGS84 **ellipsoid** height, i.e. the
    /// origin's MSL elevation plus `georeference.geoid_separation_m`.
    pub origin: GeodeticPoint,
    pub georeference: Georeference,
    pub chunks: Vec<Chunk>,
}

/// Geoid model that defines "MSL" for this pack's separation value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum GeoidModel {
    Egm96 = 1,
    Egm2008 = 2,
}

impl GeoidModel {
    pub(crate) fn from_u32(value: u32) -> Option<Self> {
        match value {
            1 => Some(Self::Egm96),
            2 => Some(Self::Egm2008),
            _ => None,
        }
    }
}

/// Identity of the pack's local ENU frame and its vertical datum link.
///
/// * `frame_id` names one surveyed origin + axis convention (x=east,
///   y=north, z=up metres about `WorldAsset::origin`). It is assigned by the
///   offline asset tool and is never zero. Packs that share a frame id must
///   share the frame.
/// * `frame_generation` starts at 1 and is incremented by the offline tool
///   whenever anything that moves geometry relative to WGS84 changes for
///   that `frame_id`: origin, convention, geoid model or separation. Zero
///   is invalid. Poses tagged with a different (id, generation) must not be
///   drawn in this pack.
/// * `geoid_separation_m` is the geoid undulation N at the origin, metres,
///   from `geoid_model`: `ellipsoid_height = msl_height + N`. Negative where
///   the geoid lies below the ellipsoid (about -32 m at SFO). One value is
///   applied over the whole pack; see `synthetic_sfo_like` for the error
///   budget that justifies this for a sparse airport extent.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Georeference {
    pub frame_id: u64,
    pub frame_generation: u64,
    pub geoid_model: GeoidModel,
    pub geoid_separation_m: f64,
}

impl Georeference {
    /// Accepted separation range, metres. Global EGM96/EGM2008 undulations
    /// span about -107 m to +86 m; anything outside this is corrupt.
    pub const SEPARATION_RANGE_M: std::ops::RangeInclusive<f64> = -110.0..=90.0;

    /// Signed correction to add to an MSL height to get ellipsoid height.
    pub fn msl_to_ellipsoid_m(&self) -> f64 {
        self.geoid_separation_m
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds3 {
    pub min: [f32; 3],
    pub max: [f32; 3],
}

#[derive(Clone, Debug, PartialEq)]
pub struct Chunk {
    pub id: u32,
    pub bounds: Bounds3,
    pub lods: Vec<Lod>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Lod {
    pub level: u8,
    pub min_distance_m: f32,
    pub max_distance_m: f32,
    pub layers: Vec<Layer>,
}

/// Triangle mesh positions are ENU metres. Indices form complete triangles.
#[derive(Clone, Debug, PartialEq)]
pub struct Mesh {
    pub positions: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct NamedMesh {
    /// Stable airport designation, e.g. `10L/28R` or `A`.
    pub name: String,
    pub mesh: Mesh,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum LightType {
    RunwayEdge = 1,
    RunwayCenterline = 2,
    TaxiwayEdge = 3,
    TaxiwayCenterline = 4,
    Approach = 5,
}

impl LightType {
    pub(crate) fn from_byte(value: u8) -> Option<Self> {
        match value {
            1 => Some(Self::RunwayEdge),
            2 => Some(Self::RunwayCenterline),
            3 => Some(Self::TaxiwayEdge),
            4 => Some(Self::TaxiwayCenterline),
            5 => Some(Self::Approach),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Light {
    pub position: [f32; 3],
    pub color_rgb8: [u8; 3],
    pub kind: LightType,
    pub intensity_cd: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct LightLayer {
    pub lights: Vec<Light>,
}

/// Typed layers keep render ordering and material policy out of source data.
#[derive(Clone, Debug, PartialEq)]
pub enum Layer {
    Land(Mesh),
    Water(Mesh),
    Runway(NamedMesh),
    Taxiway(NamedMesh),
    Markings(Mesh),
    Lights(LightLayer),
}

impl Layer {
    pub(crate) fn tag(&self) -> u8 {
        match self {
            Self::Land(_) => 1,
            Self::Water(_) => 2,
            Self::Runway(_) => 3,
            Self::Taxiway(_) => 4,
            Self::Markings(_) => 5,
            Self::Lights(_) => 6,
        }
    }
}
