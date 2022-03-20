use super::{Shape, ShapeType};
use crate::geometry::{geometry_traits::*, sat::SAT};
use crate::math::{math_essentials::*, Mat3};
/**
 * Oriented Bounding Box
 */
pub struct OBB {
    pub half_side: Vec3,
    pub transform: Transform,
    pub vertices: Vec<P3>,
    pub edges: Vec<EdgeIndex>,
    pub faces: Vec<FaceIndex>,
}

impl Shape for OBB {
    fn inertia_matrix(&self, mass: Real) -> Mat3 {
        panic!("inertia matrix for Plane not implemented");
    }
    fn shape_type(&self) -> ShapeType {
        ShapeType::OBB
    }
}

impl OBB {
    pub fn new(half_side: Vec3, transform: Transform) -> OBB {
        let xl = half_side.x();
        let yl = half_side.y();
        let zl = half_side.z();

        OBB {
            half_side,
            transform,
            vertices: vec![
                P3::new(xl, yl, zl),    // far top right
                P3::new(-xl, yl, zl),   // far top left
                P3::new(-xl, -yl, zl),  // far bot left
                P3::new(xl, -yl, zl),   // far bot right
                P3::new(xl, yl, -zl),   // near top right
                P3::new(-xl, yl, -zl),  // near top left
                P3::new(-xl, -yl, -zl), // near bot left
                P3::new(xl, -yl, -zl),  // near bot right
            ],
            // from top to bottom
            // in trigonometric order
            edges: vec![
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
            ],
            // by axis
            // in trigonometric order
            faces: vec![
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
            ],
        }
    }

    pub fn min(&self) -> P3 {
        let mut p = self.transform.translation.clone();
        for i in 0..self.transform.rotation.size().0 {
            let axis = self.transform.rotation.row(i);
            p -= axis * self.half_side[i];
        }
        p
    }
    // not sure
    pub fn max(&self) -> P3 {
        let mut p = self.transform.translation.clone();
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
        let p_to_obb = p - &self.transform.translation;
        for i in 0..self.transform.rotation.size().0 {
            let axis = self.transform.rotation.row(i);
            let scalar = dot(&axis, &p_to_obb);

            if scalar > self.half_side[i] || scalar < -self.half_side[i] {
                return false;
            }
        }

        true
    }

    /**
     * Closest point on the contour of the OBB to p, or inside if p is inside
     */
    pub fn closest_point(&self, p: &P3) -> P3 {
        let mut new_p = self.transform.translation.clone();
        let p_to_obb = p - &new_p;

        for i in 0..self.transform.rotation.size().0 {
            let axis = self.transform.rotation.row(i);
            let scalar = dot(&axis, &p_to_obb);
            new_p += axis * helper::clamp(scalar, -self.half_side[i], self.half_side[i]);
        }

        new_p
    }

    pub fn closest_point_on_contour(&self, p: &P3) -> P3 {
        let mut new_p = self.transform.translation.clone();
        let p_to_obb = p - &new_p;
        let mut scalars = [ZERO; 3];
        let mut i_max = 0;
        scalars[i_max] = dot(&self.transform.rotation.row(0), &p_to_obb);

        for i in 1..self.transform.rotation.size().0 {
            let axis = self.transform.rotation.row(i);
            scalars[i] = dot(&axis, &p_to_obb);

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

impl PolyhedronTrait for OBB {
    // number of vertices, edges, faces
    fn sizes(&self) -> PolyhedronSizes {
        PolyhedronSizes {
            vertices: 8,
            edges: 12,
            faces: 6,
        }
    }

    // fn sizes(&self) -> PolyhedronSizes;
    // fn local_vertices_ref(&self) -> &Vec<P3>;
    // fn local_vertex_ref(&self, vertex_idx: usize) -> &P3;
    // fn transformed_vertices(&self) -> Vec<P3>;
    // fn transformed_vertex(&self, vertex_idx: usize) -> P3;
    // // an edge is as the indices of the two points, from vertices()
    // fn edges_ref(&self) -> &Vec<EdgeIndex>;
    // // a face is described as a Vec of vertex index, the order is trigonometric
    // fn faces_ref(&self) -> &Vec<FaceIndex>;
    // // face_index is an index of the Vec<Faces> returned by faces()
    // fn face_normal_ref(&self, face_index: usize) -> &Vec3;
    // fn transform_ref(&self) -> Transform;
    fn local_vertices_ref(&self) -> &Vec<P3> {
        &self.vertices
    }
    fn local_vertex_ref(&self, vertex_idx: usize) -> &P3 {
        &self.vertices[vertex_idx]
    }
    // an edge is as the indices of the two points, from vertices()
    fn edges_ref(&self) -> &Vec<EdgeIndex> {
        &self.edges
    }
    // a face is described as a Vec of vertex index, the order is trigonometric
    fn faces_ref(&self) -> &Vec<FaceIndex> {
        &self.faces
    }

    // face_index is an index of the Vec<Faces> returned by faces()
    fn face_normal(&self, face: usize) -> Vec3 {
        let modulo = face % 2;
        let n = self.transform.rotation.row((face - modulo) / 2);
        n - n * TWO * (modulo as Real) // if face is odd, then the normal is flipped, otherwise nothing

        // match face {
        //     0 => self.transform.rotation.row(0),
        //     1 => -self.transform.rotation.row(0),
        //     2 => self.transform.rotation.row(1),
        //     3 => -self.transform.rotation.row(1),
        //     4 => self.transform.rotation.row(2),
        //     5 => -self.transform.rotation.row(2),
        //     _ => panic!(),
        // }
        // understandable version
        // if face % 2 == 0 {
        //     self.transform.rotation.row(face / 2)
        // } else {
        //     -self.transform.rotation.row((face - 1) / 2)
        // }
    }
    fn transform_ref(&self) -> &Transform {
        &self.transform
    }
    fn sat_separating_axis(&self) -> Vec<usize> {
        // vec![
        //     (self.transform_ref().rotation.row(0), 0),
        //     (self.transform_ref().rotation.row(1), 2),
        //     (self.transform_ref().rotation.row(2), 4),
        // ]

        vec![0, 2, 4]
    }
}

#[cfg(test)]
mod tests {
    use super::OBB;
    use crate::geometry::geometry_traits::PolyhedronTrait;
    use crate::math::{math_essentials::*, Mat3};
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
                    Vec3::zeros(),
                ),
            );
            let p = obb.closest_point_on_contour(&P3::new(ONE, ZERO, ZERO));
            println!("{:?}", p);
            // assert_eq!(p[0], 2.0);
            // assert_eq!(p[1], ZERO);
            // assert_eq!(p[2], ZERO);
        }
    }
    #[test]
    fn normal_face_test() {
        let obb = OBB::new(
            Vec3::new(ONE, ONE, ONE),
            Transform::new(
                Vec3::ones(),
                Rotation::composed(
                    helper::angle_2_rad(30 as Real),
                    helper::angle_2_rad(-35 as Real),
                    helper::angle_2_rad(-45 as Real),
                ),
                Vec3::new(1.125 as Real, 0.934 as Real, ZERO),
            ),
        );

        // X+
        {
            let x = obb.transform.rotation.row(0);
            let normal = obb.face_normal(0);

            assert_eq!(x[0], normal[0]);
            assert_eq!(x[1], normal[1]);
            assert_eq!(x[2], normal[2]);
        }
        // X-
        {
            let x = -obb.transform.rotation.row(0);
            let normal = obb.face_normal(1);

            assert_eq!(x[0], normal[0]);
            assert_eq!(x[1], normal[1]);
            assert_eq!(x[2], normal[2]);
        }
        // Y+
        {
            let x = obb.transform.rotation.row(1);
            let normal = obb.face_normal(2);

            assert_eq!(x[0], normal[0]);
            assert_eq!(x[1], normal[1]);
            assert_eq!(x[2], normal[2]);
        }
        // Y-
        {
            let x = -obb.transform.rotation.row(1);
            let normal = obb.face_normal(3);

            assert_eq!(x[0], normal[0]);
            assert_eq!(x[1], normal[1]);
            assert_eq!(x[2], normal[2]);
        }
        // Z+
        {
            let x = obb.transform.rotation.row(2);
            let normal = obb.face_normal(4);

            assert_eq!(x[0], normal[0]);
            assert_eq!(x[1], normal[1]);
            assert_eq!(x[2], normal[2]);
        }
        // Z-
        {
            let x = -obb.transform.rotation.row(2);
            let normal = obb.face_normal(5);

            assert_eq!(x[0], normal[0]);
            assert_eq!(x[1], normal[1]);
            assert_eq!(x[2], normal[2]);
        }
    }
    // #[test]
    // fn computed_against_stored_normal() {
    //     let obb = OBB::new(
    //         Vec3::new(ONE, ONE, ONE),
    //         Transform::new(
    //             Vec3::ones(),
    //             Rotation::composed(
    //                 helper::angle_2_rad(30 as Real),
    //                 helper::angle_2_rad(-35 as Real),
    //                 helper::angle_2_rad(-45 as Real),
    //             ),
    //             // Vec3::new(1.125 as Real, 0.934 as Real, ZERO),
    //             Vec3::zeros(),
    //         ),
    //     );
    //     println!(
    //         "{:?}",
    //         Rotation::composed(
    //             helper::angle_2_rad(30 as Real),
    //             helper::angle_2_rad(-35 as Real),
    //             helper::angle_2_rad(-45 as Real),
    //         )
    //     );

    //     for i in 0..obb.faces_ref()[0].v_i.len() {
    //         let index = obb.faces_ref()[0].v_i[i];
    //         println!(
    //             "p{:?} = {:?}",
    //             obb.faces_ref()[0].v_i[i],
    //             obb.transform.transform_vec(obb.local_vertex_ref(index))
    //         );
    //     }
    //     for i in 0..6 {
    //         let computed_normal = obb.computed_face_normal(i);
    //         let stored_normal = obb.face_normal(i);
    //         println!(
    //             "=== i {:?} computed_normal {:?} stored_normal {:?}",
    //             i,
    //             // helper::vect_angles(&computed_normal),
    //             // helper::vect_angles(&stored_normal)
    //             computed_normal,
    //             stored_normal,
    //         );
    //         assert_eq!(computed_normal[0], stored_normal[0]);
    //         assert_eq!(computed_normal[1], stored_normal[1]);
    //         assert_eq!(computed_normal[2], stored_normal[2]);
    //     }
    // }
}
