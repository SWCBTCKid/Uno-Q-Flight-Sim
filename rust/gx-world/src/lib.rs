//! Offline-prepared world assets for the flight renderer.
//!
//! The on-board reader accepts only the bounded, versioned `GXWORLD` format.
//! GIS parsing and geometry preparation belong in host-side tools.

mod codec;
mod coordinates;
mod fixture;
mod model;
mod visual;

pub use codec::{
    read_world, write_world, FormatError, Limits, COORDINATE_SYSTEM_WGS84_ENU_METRES, FORMAT_MAJOR,
    FORMAT_MINOR,
};
pub use coordinates::{EnuPoint, GeodeticPoint};
pub use fixture::{synthetic_sfo_like, SFO_FIELD_ELEVATION_MSL_M, SFO_GEOID_SEPARATION_EGM2008_M};
pub use model::{
    Bounds3, Chunk, GeoidModel, Georeference, Layer, Light, LightLayer, LightType, Lod, Mesh,
    NamedMesh, WorldAsset,
};
pub use visual::{
    read_visual, write_visual, Joint, JointDriver, JointKind, Material, Texture, TextureEncoding, VisualAsset, VisualError,
    VisualKind, VisualLimits, VisualMesh, VisualVertex, MAX_BUILDING_UV, MAX_JOINTS, MAX_JOINT_POINTS, VISUAL_FORMAT_MAJOR,
    VISUAL_FORMAT_MINOR,
};
