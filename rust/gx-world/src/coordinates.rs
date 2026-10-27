use crate::model::WorldAsset;

/// A WGS84 position. Angles are degrees and height is metres above ellipsoid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeodeticPoint {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub height_m: f64,
}

/// Local east/north/up metres relative to the world's geodetic origin.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EnuPoint {
    pub east_m: f64,
    pub north_m: f64,
    pub up_m: f64,
}

impl WorldAsset {
    /// Converts WGS84 geodetic (ellipsoid height) coordinates to the world's
    /// local ENU frame, using the workspace's single conversion in `gx-geo`.
    pub fn geodetic_to_enu(&self, point: GeodeticPoint) -> EnuPoint {
        let [east_m, north_m, up_m] = gx_geo::geodetic_to_enu(wgs84(self.origin), wgs84(point));
        EnuPoint { east_m, north_m, up_m }
    }

    /// Converts an MSL-referenced position (per this pack's geoid model) to
    /// the world's local ENU frame by applying the pack's separation.
    pub fn msl_to_enu(&self, latitude_deg: f64, longitude_deg: f64, height_msl_m: f64) -> EnuPoint {
        self.geodetic_to_enu(GeodeticPoint {
            latitude_deg,
            longitude_deg,
            height_m: height_msl_m + self.georeference.msl_to_ellipsoid_m(),
        })
    }
}

fn wgs84(p: GeodeticPoint) -> gx_geo::Wgs84Point {
    gx_geo::Wgs84Point {
        latitude_deg: p.latitude_deg,
        longitude_deg: p.longitude_deg,
        ellipsoid_height_m: p.height_m,
    }
}
