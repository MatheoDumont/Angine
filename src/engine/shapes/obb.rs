use super::{Shape, ShapeType};
use crate::math::{
    geometry_traits::*, helper, sat::SAT, Mat3, Real, Transform, Vec3, P3, TWO, ZERO,
};
/**
 * Oriented Bounding Box
 */
pub struct OBB {
    pub half_side: Vec3,
    pub transform: Transform,
}

impl Shape for OBB {
    fn inertia_matrix(&self, mass: Real) -> Mat3 {
        panic!("inertia matrix for Plane not implemented");
    }
    fn shape_type(&self) -> ShapeType {
        ShapeType::OBB
    }
    // fn transform_ref(&self) -> &Transform;
    // fn set_transform(&self, t: Transform);
}

impl OBB {
    pub fn new(half_side: Vec3, t: Transform) -> OBB {
        OBB {
            half_side: half_side,
            transform: t,
        }
    }

    pub fn min(&self) -> P3 {
        let mut p = P3::from(&self.transform.translation);
        for i in 0..self.transform.rotation.size().0 {
            let axis = self.transform.rotation.row(i);
            p -= axis * self.half_side[i];
        }
        p
    }

    pub fn max(&self) -> P3 {
        let mut p = P3::from(&self.transform.translation);
        for i in 0..self.transform.rotation.size().0 {
            let axis = self.transform.rotation.row(i);
            p += axis * self.half_side[i];
        }
        p
    }

    /**
     * is p inside the OBB
     */
    pub fn is_inside(&self, p: &P3) -> bool {
        let p_to_obb = p - &P3::from(self.transform.translation);
        for i in 0..self.transform.rotation.size().0 {
            let axis = self.transform.rotation.row(i);
            let scalar = axis.dot(&p_to_obb);

            if scalar > self.half_side[0] || scalar < -self.half_side[0] {
                return false;
            }
        }

        true
    }

    /**
     * Closest point on the contour of the OBB to p, or inside if p is inside
     */
    pub fn closest_point(&self, p: &P3) -> P3 {
        let mut new_p = P3::from(&self.transform.translation);
        let p_to_obb = p - &new_p;

        for i in 0..self.transform.rotation.size().0 {
            let axis = self.transform.rotation.row(i);
            let scalar = axis.dot(&p_to_obb);
            new_p += axis * helper::clamp(scalar, -self.half_side[i], self.half_side[i]);
        }

        new_p
    }

    pub fn closest_point_on_contour(&self, p: &P3) -> P3 {
        let mut new_p = P3::from(&self.transform.translation);
        let p_to_obb = p - &new_p;
        let mut scalars = [ZERO; 3];
        let mut i_max = 0;
        scalars[i_max] = self.transform.rotation.row(0).dot(&p_to_obb);

        for i in 1..self.transform.rotation.size().0 {
            let axis = self.transform.rotation.row(i);
            scalars[i] = axis.dot(&p_to_obb);

            if scalars[i].abs() > scalars[i_max].abs() {
                i_max = i;
            }
        }
        let mut max = self.half_side[i_max];
        if scalars[i_max] < ZERO {
            max = -max;
        }
        new_p
            + self.transform.rotation.row(i_max) * max
            + self.transform.rotation.row((i_max + 1) % 3) * scalars[(i_max + 1) % 3]
            + self.transform.rotation.row((i_max + 2) % 3) * scalars[(i_max + 2) % 3]
    }
}

impl Polyhedron for OBB {
    // number of vertices, edges, faces
    fn sizes(&self) -> (usize, usize, usize) {
        (8, 12, 6)
    }

    fn vertices(&self) -> Vec<P3> {
        let xl = self.half_side.x;
        let yl = self.half_side.y;
        let zl = self.half_side.z;

        vec![
            self.transform.transform(&P3::new(xl, yl, zl)), // far top right
            self.transform.transform(&P3::new(-xl, yl, zl)), // far top left
            self.transform.transform(&P3::new(-xl, -yl, zl)), // far bot left
            self.transform.transform(&P3::new(xl, -yl, zl)), // far bot right
            self.transform.transform(&P3::new(xl, yl, -zl)), // near top right
            self.transform.transform(&P3::new(-xl, yl, -zl)), // near top left
            self.transform.transform(&P3::new(-xl, -yl, -zl)), // near bot left
            self.transform.transform(&P3::new(xl, -yl, -zl)), // near bot right
        ]
    }
    // an edge is as the indices of the two points, from vertices()
    fn edges(&self) -> Vec<EdgeIndex> {
        // from top to bottom
        // in trigonometric order
        vec![
            // top horizontal edges
            EdgeIndex { vi1: 0, vi2: 1 },
            EdgeIndex { vi1: 1, vi2: 5 },
            EdgeIndex { vi1: 5, vi2: 4 },
            EdgeIndex { vi1: 4, vi2: 0 },
            // mid vertical edges
            EdgeIndex { vi1: 0, vi2: 3 },
            EdgeIndex { vi1: 1, vi2: 2 },
            EdgeIndex { vi1: 5, vi2: 6 },
            EdgeIndex { vi1: 4, vi2: 7 },
            // bottom horizontal edges
            EdgeIndex { vi1: 3, vi2: 2 },
            EdgeIndex { vi1: 2, vi2: 6 },
            EdgeIndex { vi1: 6, vi2: 7 },
            EdgeIndex { vi1: 7, vi2: 3 },
        ]
    }
    // a face is described as a Vec of vertex index, the order is trigonometric
    fn faces(&self) -> Vec<FaceIndex> {
        // by axis
        // in trigonometric order
        vec![
            // +X
            FaceIndex {
                v_i: vec![4, 7, 3, 0],
            },
            // -X
            FaceIndex {
                v_i: vec![1, 2, 6, 5],
            },
            // +Y
            FaceIndex {
                v_i: vec![0, 1, 5, 4],
            },
            // -Y
            FaceIndex {
                v_i: vec![3, 2, 6, 7],
            },
            // +Z
            FaceIndex {
                v_i: vec![0, 3, 2, 1],
            },
            // -Z
            FaceIndex {
                v_i: vec![5, 6, 7, 4],
            },
        ]
    }

    // face_index is an index of the Vec<Faces> returned by faces()
    fn face_normal(&self, face: usize) -> Vec3 {
        let modulo = face % 2;
        let n = self.transform.rotation.row((face - modulo) / 2);
        n - n * TWO * (modulo as Real) // if face is odd, then the normal is flipped, otherwise nothing

        // understandable version
        // if face % 2 == 0 {
        //     self.transform.rotation.row(face / 2)
        // } else {
        //     -self.transform.rotation.row((face - 1) / 2)
        // }
    }
}

impl SAT for OBB {
    fn separating_axis(&self) -> Vec<Vec3> {
        vec![
            self.transform.rotation.row(0),
            self.transform.rotation.row(1),
            self.transform.rotation.row(2),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::OBB;
    use crate::math::{Real, Rotation, Transform, Vec3, ONE, P3, ZERO};
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn OBB_is_inside() {
        // not rotated
        {
            let obb = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            // not inside
            {
                let p = P3::new(2 as Real, 2 as Real, 2 as Real);
                assert_eq!(obb.is_inside(&p), false);
            }
            // on the contour
            {
                let p = P3::new(ONE, ONE, ONE);
                assert_eq!(obb.is_inside(&p), true);
            }
            // inside
            {
                let p = P3::new(0.5, 0.5, 0.5);
                assert_eq!(obb.is_inside(&p), true);
            }

            let obb = OBB::new(Vec3::new(ONE, ONE, 2.0), Transform::identity());
            // inside

            let p = P3::new(0.5, 0.5, 3.0);
            assert_eq!(obb.is_inside(&p), false);
        }
        // rotated
        {
            let obb = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::rotation(Rotation::Z(std::f32::consts::FRAC_PI_4)),
            );
            // not inside
            {
                let p = P3::new(2 as Real, 2 as Real, 2 as Real);
                assert_eq!(obb.is_inside(&p), false);
            }
            // on the contour
            {
                assert_eq!(obb.is_inside(&obb.max()), true);

                assert_eq!(
                    obb.is_inside(&(obb.max() + Vec3::new(0.1, 0.1, 0.1))),
                    false
                );
            }
            // inside
            {
                let p = P3::new(0.2, 0.2, 0.2);
                assert_eq!(obb.is_inside(&p), true);
            }
        }
    }
    #[test]
    fn OBB_closest_point() {
        // not rotated
        {
            let obb = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let p = obb.closest_point(&P3::new(1.5, 1.5, ZERO));
            assert_eq!(p[0], ONE);
            assert_eq!(p[1], ONE);
            assert_eq!(p[2], ZERO);
        }
        // rotated
        {
            {
                let obb = OBB::new(
                    Vec3::new(ONE, ONE, ONE),
                    Transform::rotation(Rotation::Z(std::f32::consts::FRAC_PI_4)),
                );

                let p = obb.closest_point(&P3::new(ZERO, 2.0, ZERO));

                assert_eq!(p[0], ZERO);
                assert_eq!(p[1], (2 as f32).sqrt());
                assert_eq!(p[2], ZERO);
            }

            {
                let obb = OBB::new(
                    Vec3::new(ONE, ONE, 2.0),
                    Transform::rotation(Rotation::Z(std::f32::consts::FRAC_PI_4)),
                );
                let p = obb.closest_point(&P3::new(ZERO, 2.0, 2.0));

                assert_eq!(p[0], ZERO);
                assert_eq!(p[1], (2 as f32).sqrt());
                assert_eq!(p[2], 2.0);
            }
        }
    }
    #[test]
    fn OBB_closest_point_on_contour() {
        {
            let obb = OBB::new(Vec3::new(2.0, ONE, ONE), Transform::identity());
            let p = obb.closest_point_on_contour(&P3::new(ONE, -ZERO, ZERO));
            assert_eq!(p[0], 2.0);
            assert_eq!(p[1], ZERO);
            assert_eq!(p[2], ZERO);
        }
        {
            let obb = OBB::new(
                Vec3::ones(),
                Transform::new(
                    Vec3::ones(),
                    Rotation::Z(std::f32::consts::FRAC_PI_4),
                    Vec3::zero(),
                ),
            );
            let p = obb.closest_point_on_contour(&P3::new(ONE, ZERO, ZERO));
            println!("{:?}", p);
            // assert_eq!(p[0], 2.0);
            // assert_eq!(p[1], ZERO);
            // assert_eq!(p[2], ZERO);
        }
    }
}
