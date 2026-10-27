//! Small f64 vector/matrix helpers for the S4 world camera (column vectors,
//! column-major 4x4 for GL upload). World space is the pack's local ENU
//! frame: x east, y north, z up, metres.

pub type V3 = [f64; 3];

pub fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn scale(a: V3, s: f64) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

pub fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}

pub fn normalize(a: V3) -> V3 {
    let n = norm(a);
    if n > 0.0 && n.is_finite() {
        scale(a, 1.0 / n)
    } else {
        [0.0, 0.0, 0.0]
    }
}

/// 3x3 rotation, rows: `m[r][c]`.
pub type M3 = [[f64; 3]; 3];

/// Rotation matrix of a unit quaternion `[x, y, z, w]` (maps body vectors
/// to world vectors when the quaternion is body-to-world).
pub fn quat_to_m3(q: [f64; 4]) -> M3 {
    let [x, y, z, w] = q;
    [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]
}

pub fn m3_mul_v(m: &M3, v: V3) -> V3 {
    [dot(m[0], v), dot(m[1], v), dot(m[2], v)]
}

/// Column-major 4x4 (`m[col][row]`), the layout `glUniformMatrix4fv` takes
/// with `transpose = GL_FALSE`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct M4(pub [[f64; 4]; 4]);

impl M4 {
    pub fn identity() -> Self {
        let mut m = [[0.0; 4]; 4];
        for (i, col) in m.iter_mut().enumerate() {
            col[i] = 1.0;
        }
        Self(m)
    }

    pub fn mul(&self, o: &M4) -> M4 {
        let mut r = [[0.0; 4]; 4];
        for (c, col) in r.iter_mut().enumerate() {
            for (row, v) in col.iter_mut().enumerate() {
                *v = (0..4).map(|k| self.0[k][row] * o.0[c][k]).sum();
            }
        }
        M4(r)
    }

    /// Homogeneous transform of a point: (x, y, z, w) clip coordinates.
    pub fn apply(&self, p: V3) -> [f64; 4] {
        let mut out = [0.0; 4];
        for (row, v) in out.iter_mut().enumerate() {
            *v = self.0[0][row] * p[0] + self.0[1][row] * p[1] + self.0[2][row] * p[2] + self.0[3][row];
        }
        out
    }

    /// Rotation + translation: world = r * body + t.
    pub fn from_rt(r: &M3, t: V3) -> M4 {
        let mut m = M4::identity().0;
        for c in 0..3 {
            for row in 0..3 {
                m[c][row] = r[row][c];
            }
        }
        m[3][0] = t[0];
        m[3][1] = t[1];
        m[3][2] = t[2];
        M4(m)
    }

    pub fn to_f32(self) -> [f32; 16] {
        let mut out = [0.0f32; 16];
        for c in 0..4 {
            for r in 0..4 {
                out[c * 4 + r] = self.0[c][r] as f32;
            }
        }
        out
    }
}

/// OpenGL perspective (right-handed eye space looking down -z, depth -1..1).
pub fn perspective(vfov_rad: f64, aspect: f64, near: f64, far: f64) -> M4 {
    let f = 1.0 / (vfov_rad / 2.0).tan();
    let mut m = [[0.0; 4]; 4];
    m[0][0] = f / aspect;
    m[1][1] = f;
    m[2][2] = (far + near) / (near - far);
    m[2][3] = -1.0;
    m[3][2] = 2.0 * far * near / (near - far);
    M4(m)
}

/// View matrix of a camera at `eye` looking along `forward` with `up` as the
/// approximate up direction. With `eye` at the origin (camera-relative
/// rendering) the matrix holds only the rotation; translations are applied
/// in f64 by the caller before conversion to f32.
pub fn look_dir(eye: V3, forward: V3, up: V3) -> M4 {
    let f = normalize(forward);
    let s = normalize(cross(f, up));
    let u = cross(s, f);
    let mut m = M4::identity().0;
    for i in 0..3 {
        m[i][0] = s[i];
        m[i][1] = u[i];
        m[i][2] = -f[i];
    }
    m[3][0] = -dot(s, eye);
    m[3][1] = -dot(u, eye);
    m[3][2] = dot(f, eye);
    M4(m)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn identity_quaternion_and_heading() {
        let m = quat_to_m3([0.0, 0.0, 0.0, 1.0]);
        assert_eq!(m3_mul_v(&m, [1.0, 2.0, 3.0]), [1.0, 2.0, 3.0]);
        // 90 deg about z maps x to y.
        let h = std::f64::consts::FRAC_PI_4;
        let v = m3_mul_v(&quat_to_m3([0.0, 0.0, h.sin(), h.cos()]), [1.0, 0.0, 0.0]);
        assert!(close(v[0], 0.0) && close(v[1], 1.0), "{v:?}");
    }

    #[test]
    fn camera_projects_a_point_ahead_to_the_centre() {
        let view = look_dir([0.0, 0.0, 10.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
        let proj = perspective(50f64.to_radians(), 16.0 / 9.0, 0.5, 1000.0);
        let c = proj.mul(&view).apply([100.0, 0.0, 10.0]);
        assert!(close(c[0] / c[3], 0.0) && close(c[1] / c[3], 0.0), "{c:?}");
        assert!(c[3] > 0.0 && (c[2] / c[3]).abs() < 1.0);
        // A point to the left (north when looking east) is left on screen.
        let l = proj.mul(&view).apply([100.0, 10.0, 10.0]);
        assert!(l[0] / l[3] < 0.0);
        // Above is up.
        let u = proj.mul(&view).apply([100.0, 0.0, 20.0]);
        assert!(u[1] / u[3] > 0.0);
        // Behind is clipped (w < 0).
        assert!(proj.mul(&view).apply([-100.0, 0.0, 10.0])[3] < 0.0);
    }

    #[test]
    fn from_rt_matches_rotation_then_translation() {
        let r = quat_to_m3([0.0, 0.0, (0.3f64).sin(), (0.3f64).cos()]);
        let m = M4::from_rt(&r, [5.0, 6.0, 7.0]);
        let p = m.apply([1.0, 2.0, 3.0]);
        let want = add(m3_mul_v(&r, [1.0, 2.0, 3.0]), [5.0, 6.0, 7.0]);
        for i in 0..3 {
            assert!(close(p[i], want[i]));
        }
        assert_eq!(M4::identity().mul(&m), m);
    }
}
