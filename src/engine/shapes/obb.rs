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
    /**
     * source : https://meefi.pedagogie.ec-nantes.fr/meca/lexique/matrices-inertie/Formulaire.htm
     */
    fn compute_inertia_matrix(&self, mass: Real) -> Mat3 {
        let x_squared = self.half_side.x() * self.half_side.x() * 4.0;
        let y_squared = self.half_side.y() * self.half_side.y() * 4.0;
        let z_squared = self.half_side.z() * self.half_side.z() * 4.0;
        let div = mass / 12.0;

        let diag = Vector3::new(
            div * (y_squared + z_squared),
            div * (x_squared + z_squared),
            div * (x_squared + y_squared),
        );

        Mat3::diag(diag)
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::OBB
    }

    fn is_rigid_body(&self) -> bool {
        true
    }

    fn get_position(&self) -> &P3 {
        &self.transform.translation
    }
    fn get_orientation(&self) -> &Mat3 {
        &self.transform.rotation
    }

    fn get_transform(&self) -> Transform {
        self.transform
    }
    fn set_position(&mut self, p: P3) {
        self.transform.translation = p;
    }
    fn set_orientation(&mut self, o: Mat3) {
        self.transform.rotation = o;
    }

    fn set_transform(&mut self, t: Transform) {
        self.transform = t;
    }
}

impl OBB {
    pub fn new(half_side: Vec3) -> OBB {
        let xl = half_side.x();
        let yl = half_side.y();
        let zl = half_side.z();
        OBB {
            half_side,
            transform: Transform::identity(),
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
        let obb_to_p = p - &self.transform.translation;
        for i in 0..self.transform.rotation.size().0 {
            let axis = self.transform.rotation.row(i);
            let scalar = dot(&axis, &obb_to_p);

            if scalar > self.half_side[i] || scalar < -self.half_side[i] {
                return false;
            }
        }

        true
    }

    /**
     * return the projection of p onto the contour of the OBB, or p if p inside
     */
    pub fn project_point_onto_contour_or_inside(&self, p: &P3) -> P3 {
        let mut new_p = self.transform.translation.clone();
        let p_to_obb = p - &new_p;

        for i in 0..self.transform.rotation.size().0 {
            let axis = self.transform.rotation.row(i);
            let scalar = dot(&axis, &p_to_obb);
            new_p += axis * helper::clamp(scalar, -self.half_side[i], self.half_side[i]);
        }

        new_p
    }
    /**
     * return the projection of p onto the contour of the OBB, with p being in or out of the OBB.
     */
    pub fn project_point_onto_contour_only(&self, p: &P3) -> P3 {
        let n = p - &self.transform.translation;
        self.project_on_contour_in_direction(&n)
    }

    /**
     * Same as 'project_point_onto_contour_only' but with a vector.
     */
    pub fn project_on_contour_in_direction(&self, n: &Vec3) -> P3 {
        let mut scalars = [ZERO; 3];
        let mut i_max = 0;
        scalars[i_max] = dot(&self.transform.rotation.row(0), &n);
        // 0= tous différents, 1 = 2 pareils, 2 = tous pareils
        let mut cas = 0;
        let mut index_deuxiemecas_identique = 0;

        for i in 1..self.transform.rotation.size().0 {
            let axis = self.transform.rotation.row(i);
            scalars[i] = dot(&axis, &n);
            if helper::round_n_decimal(scalars[i].abs(), 6)
                == helper::round_n_decimal(scalars[i_max].abs(), 6)
            {
                if cas == 1 {
                    cas = 2;
                } else {
                    index_deuxiemecas_identique = i;
                    cas = 1;
                }
            } else if scalars[i].abs() > scalars[i_max].abs() {
                i_max = i;
            }
        }
        if cas == 0 {
            let i = (i_max + 1) % 3;
            let j = (i_max + 2) % 3;
            self.transform.translation
                + self.transform.rotation.row(i_max)
                    * self.half_side[i_max]
                    * helper::sign(scalars[i_max])
                + self.transform.rotation.row(i)
                    * helper::clamp(scalars[i], -self.half_side[i], self.half_side[i])
                + self.transform.rotation.row(j)
                    * helper::clamp(scalars[j], -self.half_side[j], self.half_side[j])
        } else if cas == 1 {
            let mut j = (i_max + 1) % 3;
            if j == index_deuxiemecas_identique {
                j = (i_max + 2) % 3;
            }
            self.transform.translation
                + self.transform.rotation.row(i_max)
                    * self.half_side[i_max]
                    * helper::sign(scalars[i_max])
                + self.transform.rotation.row(index_deuxiemecas_identique)
                    * self.half_side[index_deuxiemecas_identique]
                    * helper::sign(scalars[index_deuxiemecas_identique])
                + self.transform.rotation.row(j)
                    * helper::clamp(scalars[j], -self.half_side[j], self.half_side[j])
        } else {
            self.transform.translation
                + self.transform.rotation.row(0) * self.half_side[0] * helper::sign(scalars[0])
                + self.transform.rotation.row(1) * self.half_side[1] * helper::sign(scalars[1])
                + self.transform.rotation.row(2) * self.half_side[2] * helper::sign(scalars[2])
        }
    }

    /**
     * Return the distance from the center of OBB to the point on the contour in the direction of the normal 'n'.
     */
    pub fn contour_distance(&self, n: &Vec3) -> Real {
        self.half_side[0] * dot(&n, &self.transform.rotation.row(0)).abs()
            + self.half_side[1] * dot(&n, &self.transform.rotation.row(1)).abs()
            + self.half_side[2] * dot(&n, &self.transform.rotation.row(2)).abs()
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
        vec![0, 2, 4]
    }
    fn adjacent_faces(&self, face_index: usize) -> Vec<usize> {
        match face_index {
            0 => vec![3, 4, 2, 5],
            1 => vec![3, 5, 2, 4],
            2 => vec![5, 0, 4, 1],
            3 => vec![5, 1, 4, 0],
            4 => vec![2, 0, 3, 1],
            5 => vec![3, 0, 2, 1],
            _ => panic!("shapes::obb:adjacent_faces() === face_index should be between 0 and 5 !"),
        }
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
            let obb = OBB::new(Vec3::new(ONE, ONE, ONE));
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

            let obb = OBB::new(Vec3::new(ONE, ONE, 2.0));
            // inside

            let p = P3::new(0.5, 0.5, 3.0);
            assert_eq!(obb.is_inside(&p), false);
        }
        // rotated
        {
            let mut obb = OBB::new(Vec3::new(ONE, ONE, ONE));
            obb.transform = Transform::rotation(Rotation::Z(std::f32::consts::FRAC_PI_4));
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
    fn OBB_project_point_onto_contour() {
        // project_point_onto_contour_or_inside
        {
            // not rotated
            {
                let obb = OBB::new(Vec3::new(ONE, ONE, ONE));
                let p = obb.project_point_onto_contour_or_inside(&P3::new(1.5, 1.5, ZERO));
                assert_eq!(p[0], ONE);
                assert_eq!(p[1], ONE);
                assert_eq!(p[2], ZERO);
            }
            // rotated
            {
                {
                    let mut obb = OBB::new(Vec3::new(ONE, ONE, ONE));
                    obb.transform = Transform::rotation(Rotation::Z(std::f32::consts::FRAC_PI_4));

                    let p = obb.project_point_onto_contour_or_inside(&P3::new(ZERO, 2.0, ZERO));

                    assert_eq!(p[0], ZERO);
                    assert_eq!(p[1], (2 as f32).sqrt());
                    assert_eq!(p[2], ZERO);
                }

                {
                    let mut obb = OBB::new(Vec3::new(ONE, ONE, 2.0));
                    obb.transform = Transform::rotation(Rotation::Z(std::f32::consts::FRAC_PI_4));
                    let p = obb.project_point_onto_contour_or_inside(&P3::new(ZERO, 2.0, 2.0));

                    assert_eq!(p[0], ZERO);
                    assert_eq!(p[1], (2 as f32).sqrt());
                    assert_eq!(p[2], 2.0);
                }
            }
        }
        // project_point_onto_contour_only()
        {
            {
                let obb = OBB::new(Vec3::new(2.0, ONE, ONE));
                let p = obb.project_point_onto_contour_only(&P3::new(ONE, ZERO, ZERO));
                assert_eq!(p[0], 2.0);
                assert_eq!(p[1], ZERO);
                assert_eq!(p[2], ZERO);
            }

            // edge
            {
                let obb = OBB::new(Vec3::value(ONE));
                let p = obb.project_on_contour_in_direction(&Vec3::new(ONE, ONE, ZERO));
                assert_eq!(p[0], ONE);
                assert_eq!(p[1], ONE);
                assert_eq!(p[2], ZERO);
            }
            // and direction non normalisé
            {
                let obb = OBB::new(Vec3::value(ONE));
                let p = obb.project_on_contour_in_direction(&Vec3::new(TWO, TWO, ZERO));
                assert_eq!(p[0], ONE);
                assert_eq!(p[1], ONE);
                assert_eq!(p[2], ZERO);
            }

            // corner
            {
                let obb = OBB::new(Vec3::value(ONE));
                let p = obb.project_on_contour_in_direction(&Vec3::new(ONE, ONE, ONE));
                assert_eq!(p[0], ONE);
                assert_eq!(p[1], ONE);
                assert_eq!(p[2], ONE);
            }

            // with rectangle
            {
                let obb = OBB::new(Vec3::new(TWO, ONE, ONE));
                let p = obb.project_on_contour_in_direction(&Vec3::new(TWO, ONE, ZERO));
                assert_eq!(p[0], TWO);
                assert_eq!(p[1], ONE);
                assert_eq!(p[2], ZERO);
            }

            // rotated
            {
                let t = Transform::rotation(Rotation::Z(helper::angle_2_rad(45.0)));
                let pp = t.transform(&P3::new(-ONE, -ONE, -ONE));
                let mut obb = OBB::new(Vec3::new(ONE, ONE, ONE));
                obb.transform = t;
                let p = obb.project_on_contour_in_direction(&Vec3::new(ZERO, -ONE, ZERO));

                assert_eq!(p[0], pp.x());
                assert_eq!(p[1], pp.y());
                assert_eq!(p[2], ZERO);
            }
        }
    }

    #[test]
    fn normal_face_test() {
        let mut obb = OBB::new(Vec3::new(ONE, ONE, ONE));
        obb.transform = Transform::new(
            Vec3::ones(),
            Rotation::composed(
                helper::angle_2_rad(30 as Real),
                helper::angle_2_rad(-35 as Real),
                helper::angle_2_rad(-45 as Real),
            ),
            Vec3::new(1.125 as Real, 0.934 as Real, ZERO),
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
}
