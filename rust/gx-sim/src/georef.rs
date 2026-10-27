use crate::{GeodeticDatum, GeodeticPosition, LocalEnuPose};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeoreferenceError {
    InvalidFrame,
    InvalidOrigin,
    InvalidPosition,
}

/// Explicit seam from JSBSim's MSL altitude to an asset's WGS84 local ENU
/// frame. The signed MSL-to-ellipsoid correction must come from reviewed world
/// metadata; it is never inferred from AGL.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Wgs84EnuFrame {
    frame_id: u64,
    frame_generation: u64,
    origin: GeodeticPosition,
    msl_to_ellipsoid_m: f64,
}

impl Wgs84EnuFrame {
    pub fn new(
        frame_id: u64,
        frame_generation: u64,
        origin: GeodeticPosition,
        msl_to_ellipsoid_m: f64,
    ) -> Result<Self, GeoreferenceError> {
        if frame_id == 0 { return Err(GeoreferenceError::InvalidFrame); }
        if !valid_geodetic(origin) || !msl_to_ellipsoid_m.is_finite() {
            return Err(GeoreferenceError::InvalidOrigin);
        }
        Ok(Self { frame_id, frame_generation, origin, msl_to_ellipsoid_m })
    }

    pub fn origin(&self) -> GeodeticPosition { self.origin }
    pub fn frame_id(&self) -> u64 { self.frame_id }
    pub fn frame_generation(&self) -> u64 { self.frame_generation }

    /// Convert an explicitly MSL-referenced position and NED velocity.
    pub fn project_msl(
        &self,
        latitude_deg: f64,
        longitude_deg: f64,
        height_msl_m: f64,
        velocity_ned_mps: [f64; 3],
        attitude_rph_rad: [f64; 3],
    ) -> Result<(GeodeticPosition, LocalEnuPose), GeoreferenceError> {
        let geodetic = GeodeticPosition {
            datum: GeodeticDatum::Wgs84Ellipsoid,
            latitude_deg,
            longitude_deg,
            ellipsoid_height_m: height_msl_m + self.msl_to_ellipsoid_m,
        };
        if !valid_geodetic(geodetic)
            || !velocity_ned_mps.into_iter().all(f64::is_finite)
            || !attitude_rph_rad.into_iter().all(f64::is_finite)
        {
            return Err(GeoreferenceError::InvalidPosition);
        }
        let position_m = gx_geo::geodetic_to_enu(wgs84(self.origin), wgs84(geodetic));
        Ok((geodetic, LocalEnuPose {
            frame_id: self.frame_id,
            frame_generation: self.frame_generation,
            position_m,
            velocity_mps: [velocity_ned_mps[1], velocity_ned_mps[0], -velocity_ned_mps[2]],
            orientation_xyzw: body_to_enu_quaternion(attitude_rph_rad),
        }))
    }
}

fn body_to_enu_quaternion([phi, theta, psi]: [f64; 3]) -> [f64; 4] {
    let (sr, cr) = phi.sin_cos();
    let (st, ct) = theta.sin_cos();
    let (sy, cy) = psi.sin_cos();
    let body_to_ned = [
        [ct * cy, sr * st * cy - cr * sy, cr * st * cy + sr * sy],
        [ct * sy, sr * st * sy + cr * cy, cr * st * sy - sr * cy],
        [-st, sr * ct, cr * ct],
    ];
    let m = [body_to_ned[1], body_to_ned[0], [
        -body_to_ned[2][0], -body_to_ned[2][1], -body_to_ned[2][2],
    ]];
    matrix_quaternion(m)
}

fn matrix_quaternion(m: [[f64; 3]; 3]) -> [f64; 4] {
    let trace = m[0][0] + m[1][1] + m[2][2];
    let (x, y, z, w) = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
         (m[1][0] - m[0][1]) / s, 0.25 * s)
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
        (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s,
         (m[2][1] - m[1][2]) / s)
    } else if m[1][1] > m[2][2] {
        let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
        ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s,
         (m[0][2] - m[2][0]) / s)
    } else {
        let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
        ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s,
         (m[1][0] - m[0][1]) / s)
    };
    let norm = (x * x + y * y + z * z + w * w).sqrt();
    [x / norm, y / norm, z / norm, w / norm]
}

fn valid_geodetic(p: GeodeticPosition) -> bool {
    p.datum == GeodeticDatum::Wgs84Ellipsoid && wgs84(p).is_valid()
}

fn wgs84(p: GeodeticPosition) -> gx_geo::Wgs84Point {
    gx_geo::Wgs84Point {
        latitude_deg: p.latitude_deg,
        longitude_deg: p.longitude_deg,
        ellipsoid_height_m: p.ellipsoid_height_m,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sfo_frame() -> Wgs84EnuFrame {
        Wgs84EnuFrame::new(0x5346_4f, 7, GeodeticPosition {
            datum: GeodeticDatum::Wgs84Ellipsoid,
            latitude_deg: 37.618_805_6,
            longitude_deg: -122.375_416_7,
            ellipsoid_height_m: -24.0,
        }, -28.0).unwrap()
    }

    #[test]
    fn origin_maps_to_zero_and_names_ellipsoid_height() {
        let frame = sfo_frame();
        let (geo, local) = frame.project_msl(
            frame.origin.latitude_deg, frame.origin.longitude_deg, 4.0,
            [1.0, 2.0, 3.0], [0.0; 3]
        ).unwrap();
        assert_eq!(geo.datum, GeodeticDatum::Wgs84Ellipsoid);
        assert_eq!(geo.ellipsoid_height_m, -24.0);
        assert!(local.position_m.into_iter().all(|v| v.abs() < 1e-8));
        assert_eq!(local.velocity_mps, [2.0, 1.0, -3.0]);
        let q_norm = local.orientation_xyzw.into_iter().map(|v| v * v).sum::<f64>();
        assert!((q_norm - 1.0).abs() < 1e-12);
        assert_eq!((local.frame_id, local.frame_generation), (0x5346_4f, 7));
    }

    #[test]
    fn nonzero_offset_has_expected_enu_direction_and_scale() {
        let frame = sfo_frame();
        let (_, local) = frame.project_msl(
            frame.origin.latitude_deg + 0.001,
            frame.origin.longitude_deg + 0.001,
            14.0,
            [0.0; 3],
            [0.1, -0.2, 0.3],
        ).unwrap();
        assert!((87.0..90.0).contains(&local.position_m[0]));
        assert!((110.0..112.0).contains(&local.position_m[1]));
        assert!((9.0..11.0).contains(&local.position_m[2]));
    }

    #[test]
    fn invalid_or_implicit_frame_is_rejected() {
        let origin = GeodeticPosition { latitude_deg: 37.0, longitude_deg: -122.0, ..Default::default() };
        assert_eq!(Wgs84EnuFrame::new(0, 0, origin, 0.0), Err(GeoreferenceError::InvalidFrame));
        assert_eq!(sfo_frame().project_msl(91.0, 0.0, 0.0, [0.0; 3], [0.0; 3]), Err(GeoreferenceError::InvalidPosition));
    }

    /// Hand-computed reference (see gx-geo): from (0N, 0E, 0 m ellipsoid),
    /// the point (0N, 90E) is at ENU (a, 0, -a). An MSL input of 10 m with a
    /// -10 m separation lands on the ellipsoid, so the reference holds.
    #[test]
    fn project_msl_matches_hand_computed_reference() {
        let frame = Wgs84EnuFrame::new(1, 1, GeodeticPosition::default(), -10.0).unwrap();
        let (geo, local) = frame.project_msl(0.0, 90.0, 10.0, [0.0; 3], [0.0; 3]).unwrap();
        assert_eq!(geo.ellipsoid_height_m, 0.0);
        let a = gx_geo::WGS84_A_M;
        for (axis, expected) in [a, 0.0, -a].into_iter().enumerate() {
            assert!((local.position_m[axis] - expected).abs() < 1e-6, "{:?}", local.position_m);
        }
    }
}
