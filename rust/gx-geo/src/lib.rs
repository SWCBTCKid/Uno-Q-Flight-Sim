//! The one WGS84 geodetic -> ECEF -> local ENU conversion in the workspace.
//!
//! Both `gx-sim` (FDM pose) and `gx-world` (asset frame) call this crate, so
//! the simulation and the renderer cannot drift apart (ISS-036). It has no
//! dependencies and knows nothing about rendering or simulation types.
//!
//! Heights here are always **WGS84 ellipsoid** heights. MSL heights must be
//! converted by the caller with a named geoid separation first.

/// WGS84 semi-major axis, metres.
pub const WGS84_A_M: f64 = 6_378_137.0;
/// WGS84 first eccentricity squared, `f * (2 - f)` with `f = 1/298.257223563`.
pub const WGS84_E2: f64 = 6.694_379_990_14e-3;

/// A WGS84 position: degrees and metres above the ellipsoid.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Wgs84Point {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub ellipsoid_height_m: f64,
}

impl Wgs84Point {
    /// Finite, latitude in [-90, 90], longitude in [-180, 180], finite height.
    pub fn is_valid(self) -> bool {
        self.latitude_deg.is_finite()
            && (-90.0..=90.0).contains(&self.latitude_deg)
            && self.longitude_deg.is_finite()
            && (-180.0..=180.0).contains(&self.longitude_deg)
            && self.ellipsoid_height_m.is_finite()
    }
}

/// Earth-centred, earth-fixed metres.
pub fn geodetic_to_ecef(p: Wgs84Point) -> [f64; 3] {
    let lat = p.latitude_deg.to_radians();
    let lon = p.longitude_deg.to_radians();
    let n = WGS84_A_M / (1.0 - WGS84_E2 * lat.sin().powi(2)).sqrt();
    let h = p.ellipsoid_height_m;
    [
        (n + h) * lat.cos() * lon.cos(),
        (n + h) * lat.cos() * lon.sin(),
        (n * (1.0 - WGS84_E2) + h) * lat.sin(),
    ]
}

/// East/north/up metres of `point` in the tangent frame at `origin`.
pub fn geodetic_to_enu(origin: Wgs84Point, point: Wgs84Point) -> [f64; 3] {
    let o = geodetic_to_ecef(origin);
    let p = geodetic_to_ecef(point);
    let (dx, dy, dz) = (p[0] - o[0], p[1] - o[1], p[2] - o[2]);
    let (slat, clat) = origin.latitude_deg.to_radians().sin_cos();
    let (slon, clon) = origin.longitude_deg.to_radians().sin_cos();
    [
        -slon * dx + clon * dy,
        -slat * clon * dx - slat * slon * dy + clat * dz,
        clat * clon * dx + clat * slon * dy + slat * dz,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    const B_M: f64 = 6_356_752.314_245; // WGS84 semi-minor axis, a * sqrt(1 - e2)

    fn p(latitude_deg: f64, longitude_deg: f64, ellipsoid_height_m: f64) -> Wgs84Point {
        Wgs84Point { latitude_deg, longitude_deg, ellipsoid_height_m }
    }

    fn near(actual: [f64; 3], expected: [f64; 3], tol: f64) {
        for axis in 0..3 {
            assert!(
                (actual[axis] - expected[axis]).abs() < tol,
                "axis {axis}: {actual:?} vs {expected:?}"
            );
        }
    }

    #[test]
    fn ecef_reference_points() {
        near(geodetic_to_ecef(p(0.0, 0.0, 0.0)), [WGS84_A_M, 0.0, 0.0], 1e-6);
        near(geodetic_to_ecef(p(0.0, 90.0, 100.0)), [0.0, WGS84_A_M + 100.0, 0.0], 1e-6);
        near(geodetic_to_ecef(p(90.0, 0.0, 0.0)), [0.0, 0.0, B_M], 1e-6);
    }

    /// Hand-computed: from (0N, 0E, 0 m) the point (0N, 90E, 0 m) is at
    /// ECEF delta (-a, +a, 0); the equatorial ENU basis at lon 0 is
    /// east = +Y, north = +Z, up = +X, so ENU = (a, 0, -a).
    #[test]
    fn enu_reference_equator_quarter_turn() {
        near(geodetic_to_enu(p(0.0, 0.0, 0.0), p(0.0, 90.0, 0.0)), [WGS84_A_M, 0.0, -WGS84_A_M], 1e-6);
    }

    /// Hand-computed: from the north pole (lon 0) the point (0N, 0E, 0 m)
    /// has ECEF delta (a, 0, -b); at the pole north = -X, up = +Z, so
    /// ENU = (0, -a, -b).
    #[test]
    fn enu_reference_north_pole() {
        near(geodetic_to_enu(p(90.0, 0.0, 0.0), p(0.0, 0.0, 0.0)), [0.0, -WGS84_A_M, -B_M], 1e-6);
    }

    #[test]
    fn enu_origin_is_zero_and_height_is_up() {
        let o = p(37.618_805_6, -122.375_416_7, -28.275_3);
        near(geodetic_to_enu(o, o), [0.0; 3], 1e-8);
        near(geodetic_to_enu(o, Wgs84Point { ellipsoid_height_m: o.ellipsoid_height_m + 10.0, ..o }), [0.0, 0.0, 10.0], 1e-6);
    }

    /// Small northward step at SFO against the meridian radius of curvature
    /// M = a(1-e2)/(1-e2 sin^2 lat)^1.5; second-order terms are < 1 mm.
    #[test]
    fn enu_small_north_step_matches_meridian_radius() {
        let o = p(37.618_805_6, -122.375_416_7, 0.0);
        let dlat = 0.001_f64;
        let s = o.latitude_deg.to_radians().sin();
        let m = WGS84_A_M * (1.0 - WGS84_E2) / (1.0 - WGS84_E2 * s * s).powf(1.5);
        let enu = geodetic_to_enu(o, Wgs84Point { latitude_deg: o.latitude_deg + dlat, ..o });
        assert!(enu[0].abs() < 1e-6);
        assert!((enu[1] - m * dlat.to_radians()).abs() < 1e-3, "{enu:?}");
        assert!(enu[2] < 0.0 && enu[2] > -0.01, "{enu:?}");
    }

    #[test]
    fn validity() {
        assert!(p(37.0, -122.0, 0.0).is_valid());
        assert!(!p(91.0, 0.0, 0.0).is_valid());
        assert!(!p(0.0, 181.0, 0.0).is_valid());
        assert!(!p(0.0, 0.0, f64::NAN).is_valid());
    }
}
