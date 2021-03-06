use super::{Mat3, Quaternion, Real, Vec3, ONE, ZERO};

pub struct RotationMatrix;

impl RotationMatrix {
    /**
     * All angles are in radian
     */
    pub fn X(pitch: Real) -> Mat3 {
        let c = pitch.cos();
        let s = pitch.sin();
        Mat3::from_array([[ONE, ZERO, ZERO], [ZERO, c, -s], [ZERO, s, c]])
    }
    pub fn Y(yaw: Real) -> Mat3 {
        let c = yaw.cos();
        let s = yaw.sin();
        Mat3::from_array([[c, ZERO, s], [ZERO, ONE, ZERO], [-s, ZERO, c]])
    }
    pub fn Z(roll: Real) -> Mat3 {
        let c = roll.cos();
        let s = roll.sin();
        Mat3::from_array([[c, -s, ZERO], [s, c, ZERO], [ZERO, ZERO, ONE]])
    }

    pub fn composed(x_axis_rad: Real, y_axis_rad: Real, z_axis_rad: Real) -> Mat3 {
        RotationMatrix::Z(z_axis_rad)
            * RotationMatrix::X(x_axis_rad)
            * RotationMatrix::Y(y_axis_rad)
    }

    pub fn axis_angle(normalized_axis: Vec3, rad: Real) -> Mat3 {
        let c = rad.cos();
        let s = rad.sin();
        let t = ONE - c;
        let x = normalized_axis.x();
        let y = normalized_axis.y();
        let z = normalized_axis.z();

        let op1 = t * x * y + s * z;
        let op2 = t * x * z + s * y;
        let op3 = t * y * z + s * x;

        Mat3::from_array([
            [t * x * x + c, op1, op2],
            [op1, t * y * y + c, op3],
            [op2, op3, t * z * z + c],
        ])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq;

    

    #[test]
    fn rotation_methods_test() {
        let rad = std::f32::consts::FRAC_PI_2;
        // Z rotation
        {
            let v = Vec3::new(ONE, ZERO, ZERO);
            let rotated = RotationMatrix::Z(rad) * v;
            // println!("{:?}", rotated);
            assert_approx_eq!(rotated.x(), ZERO, 1.0e-6);
            assert_approx_eq!(rotated.y(), ONE, 1.0e-6);
            assert_approx_eq!(rotated.z(), ZERO, 1.0e-6);
        }
        // Y rotation
        {
            let v = Vec3::new(ZERO, ZERO, ONE);
            let rotated = RotationMatrix::Y(rad) * v;
            // println!("{:?}", rotated);
            assert_approx_eq!(rotated.x(), ONE, 1.0e-6);
            assert_approx_eq!(rotated.y(), ZERO, 1.0e-6);
            assert_approx_eq!(rotated.z(), ZERO, 1.0e-6);
        }
        // X rotation
        {
            let v = Vec3::new(ZERO, ONE, ZERO);
            let rotated = RotationMatrix::X(rad) * v;
            // println!("{:?}", rotated);
            assert_approx_eq!(rotated.x(), ZERO, 1.0e-6);
            assert_approx_eq!(rotated.y(), ZERO, 1.0e-6);
            assert_approx_eq!(rotated.z(), ONE, 1.0e-6);
        }

        // composed
        {
            let v = Vec3::new(ONE, ZERO, ZERO);
            let rotated = RotationMatrix::composed(rad, rad, rad) * v;
            // println!("{:?}", rotated);
            assert_approx_eq!(rotated.x(), -ONE, 1.0e-6);
            assert_approx_eq!(rotated.y(), ZERO, 1.0e-6);
            assert_approx_eq!(rotated.z(), ZERO, 1.0e-6);
        }

        // axis angle
        {
            let z_axis = Vec3::new(ZERO, ZERO, ONE);
            let v = Vec3::new(ONE, ZERO, ZERO);
            let rotated = RotationMatrix::axis_angle(z_axis, rad) * v;
            // println!("{:?}", rotated);
            assert_approx_eq!(rotated.x(), ZERO, 1.0e-6);
            assert_approx_eq!(rotated.y(), ONE, 1.0e-6);
            assert_approx_eq!(rotated.z(), ZERO, 1.0e-6);
        }
    }
}
