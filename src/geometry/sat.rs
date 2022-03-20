use super::geometry_traits::*;
use crate::math::math_essentials::*;
/**
 * Implementation of the Separating Axis Theorem
 */

pub trait SAT {
    fn separating_axis(&self) -> Vec<Vec3>;
}

pub struct Projection {
    min: Real,
    max: Real,
}

// distance, which face of the shape, the axis
#[derive(Debug)]
pub struct Face {
    distance: Real,
    axis: Vec3,
    face_index: usize,
}
// distance, edge of A, edge of B, the axis
#[derive(Debug)]
pub struct Edge {
    distance: Real,
    axis: Vec3,
    edge_a_index: usize,
    edge_b_index: usize,
}

pub struct SAT2DResult {
    // pub minimum_translation: Real,
    // pub minimum_axis: Vec3,
    pub face_A: Face,
    pub face_B: Face,
}
#[derive(Debug)]
pub struct SAT3DResult {
    // pub minimum_translation: Real,
    // pub minimum_axis: Vec3,
    pub face_A: Face,
    pub face_B: Face,
    pub edge: Edge,
}

fn project(axis: &Vec3, vertices: &Vec<P3>) -> Projection {
    let mut min = dot(&axis, &vertices[0]);
    let mut max = min;

    for i in 1..vertices.len() {
        let x = dot(&axis, &vertices[i]);
        if x > max {
            max = x;
        } else if x < min {
            min = x;
        }
    }

    Projection { min: min, max: max }
}

fn overlapping_or_touching(p1: &Projection, p2: &Projection) -> Option<Real> {
    if p1.max >= p2.min && p2.max >= p1.min {
        if p1.max > p2.max {
            Some(p2.max - p1.min)
        } else {
            Some(p1.max - p2.min)
        }
    } else {
        None
    }
}

pub fn sat_2D(
    shape1_vertices: &Vec<P3>,
    axis_shape1: &Vec<Vec3>,
    shape2_vertices: &Vec<P3>,
    axis_shape2: &Vec<Vec3>,
) -> Option<SAT2DResult> {
    let mut distance = f32::MAX;
    let mut best_axis = &Vec3::zeros();
    let mut idx_best_axis = 0;

    // face of A
    for axis_idx in 0..axis_shape1.len() {
        let axis = &axis_shape1[axis_idx];
        let proj1 = project(axis, &shape1_vertices);
        let proj2 = project(axis, &shape2_vertices);

        if let Some(x) = overlapping_or_touching(&proj1, &proj2) {
            if x < distance {
                distance = x;
                best_axis = axis;
                idx_best_axis = axis_idx;
            }
        } else {
            return None;
        }
    }

    let face_A = Face {
        distance,
        axis: best_axis.clone(),
        face_index: idx_best_axis,
    };

    distance = f32::MAX;

    // face of B
    for axis_idx in 0..axis_shape2.len() {
        let axis = &axis_shape2[axis_idx];
        let proj1 = project(axis, &shape1_vertices);
        let proj2 = project(axis, &shape2_vertices);

        if let Some(x) = overlapping_or_touching(&proj1, &proj2) {
            if x < distance {
                distance = x;
                best_axis = axis;
                idx_best_axis = axis_idx;
            }
        } else {
            return None;
        }
    }

    let face_B = Face {
        distance,
        axis: best_axis.clone(),
        face_index: idx_best_axis,
    };

    Some(SAT2DResult { face_A, face_B })
}

pub fn closest_projection_vec(
    normal: &Vec3,
    to_points: &Vec<P3>,
    from_point: &P3,
) -> (Real, usize) {
    let mut smallest_distance = f32::MAX;
    let mut closest_point_index = 0;
    for i in 0..to_points.len() {
        let v = &to_points[i] - from_point;
        let x = dot(&normal, &v);
        if x < smallest_distance {
            smallest_distance = x;
            closest_point_index = i;
        }
    }

    (smallest_distance, closest_point_index)
}

fn face_intersection<T: PolyhedronTrait>(shape_A: &T, shape_B: &T) -> Face {
    let mut best_face = Face {
        distance: f32::MAX,
        axis: Vec3::zeros(),
        face_index: 0,
    };
    let transformed_vertices_A = shape_A.transformed_vertices();
    let transformed_vertices_B = shape_B.transformed_vertices();

    for face_index in 0..shape_A.sizes().faces {
        // point on the plane
        let face_indices = &shape_A.faces_ref()[face_index];
        let point_on_face = transformed_vertices_A[face_indices.v_i[0]];
        let normal = shape_A.face_normal(face_index);

        // closest point to the normal
        let result = closest_projection_vec(&normal, &transformed_vertices_B, &point_on_face);

        // if result.0 > ZERO {
        //     println!(
        //         "face_index:{:?} normal:{:?} norm:{:?} transformed_vertices_B:{:?} distance:{:?} ",
        //         face_index,
        //         normal,
        //         normal.norm(),
        //         transformed_vertices_B[result.1],
        //         result.0
        //     );
        //     return best_face;
        // }

        if result.0 < best_face.distance {
            best_face.distance = result.0;
            best_face.axis = normal;
            best_face.face_index = face_index;
        }
    }

    best_face
}

pub fn sat_3D<T: PolyhedronTrait>(shape_A: &T, shape_B: &T) -> Option<SAT3DResult> {
    println!("=====================================================");
    let vertices_A = shape_A.transformed_vertices();
    let vertices_B = shape_B.transformed_vertices();

    // ======================== Normals of A ========================
    let mut best_face_A = Face {
        distance: f32::MAX,
        axis: Vec3::zeros(),
        face_index: 0,
    };
    for face_index in shape_A.sat_separating_axis() {
        let normal = shape_A.face_normal(face_index);

        let proj1 = project(&normal, &vertices_A);
        let proj2 = project(&normal, &vertices_B);

        if let Some(x) = overlapping_or_touching(&proj1, &proj2) {
            if x < best_face_A.distance {
                best_face_A.distance = x;
                best_face_A.axis = normal;
                best_face_A.face_index = face_index;
            }
        } else {
            return None;
        }
    }
    // ======================== Normals of B ========================
    let mut best_face_B = Face {
        distance: f32::MAX,
        axis: Vec3::zeros(),
        face_index: 0,
    };
    for face_index in shape_B.sat_separating_axis() {
        let normal = shape_B.face_normal(face_index);

        let proj1 = project(&normal, &vertices_A);
        let proj2 = project(&normal, &vertices_B);

        if let Some(x) = overlapping_or_touching(&proj1, &proj2) {
            if x < best_face_B.distance {
                best_face_B.distance = x;
                best_face_B.axis = normal;
                best_face_B.face_index = face_index;
            }
        } else {
            return None;
        }
    }

    // ======================== Cross product between edges of A and B ========================
    let mut best_edge = Edge {
        distance: f32::MAX,
        axis: Vec3::zeros(),
        edge_a_index: 0,
        edge_b_index: 0,
    };
    for edge1_index in 0..shape_A.sizes().edges {
        let edge1 = &shape_A.edges_ref()[edge1_index];
        for edge2_index in 0..shape_B.sizes().edges {
            let edge2 = &shape_B.edges_ref()[edge2_index];
            let direction_edge_A = vertices_A[edge1.vi1] - vertices_A[edge1.vi2];
            let direction_edge_B = vertices_B[edge2.vi1] - vertices_B[edge2.vi2];

            let mut cross = cross(&direction_edge_A, &direction_edge_B);
            let norm = magnitude(&cross);

            if norm == ZERO {
                continue;
            }

            cross /= norm;

            // reoriente la normale pour partir de A
            let r = &vertices_A[edge1.vi1] - &shape_A.transform_ref().position();
            if dot(&cross, &r) < ZERO {
                cross = -cross;
            }

            let p1 = project(&cross, &vertices_A);
            let p2 = project(&cross, &vertices_B);

            if let Some(x) = overlapping_or_touching(&p1, &p2) {
                if x < best_edge.distance {
                    best_edge.distance = x;
                    best_edge.axis = cross;
                    best_edge.edge_a_index = edge1_index;
                    best_edge.edge_b_index = edge2_index;
                }
            } else {
                return None;
            }
        }
    }

    Some(SAT3DResult {
        face_A: best_face_A,
        face_B: best_face_B,
        edge: best_edge,
    })
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::engine::shapes::OBB;
    use crate::math::{helper, Rotation, Transform, ONE, ZERO};
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn test_overlapping_or_touching() {
        {
            let p1 = Projection {
                min: 1 as Real,
                max: 3 as Real,
            };
            let p2 = Projection {
                min: 2.5 as Real,
                max: 4 as Real,
            };

            assert_eq!(overlapping_or_touching(&p1, &p2), Some(0.5));
        }

        {
            let p1 = Projection {
                min: 1 as Real,
                max: 3 as Real,
            };
            let p2 = Projection {
                min: 2.5 as Real,
                max: 4 as Real,
            };

            assert_eq!(overlapping_or_touching(&p2, &p1), Some(0.5));
        }

        {
            let p1 = Projection {
                min: 1 as Real,
                max: 3 as Real,
            };
            let p2 = Projection {
                min: 2.5 as Real,
                max: 7 as Real,
            };

            assert_eq!(overlapping_or_touching(&p2, &p1), Some(0.5));
        }
        // touching but not overlapping
        {
            let p1 = Projection {
                min: 1 as Real,
                max: 3 as Real,
            };
            let p2 = Projection {
                min: 3 as Real,
                max: 7 as Real,
            };

            assert_eq!(overlapping_or_touching(&p2, &p1), Some(ZERO));
        }

        // nothing
        {
            let p1 = Projection {
                min: 1 as Real,
                max: 3 as Real,
            };
            let p2 = Projection {
                min: 3.1 as Real,
                max: 7 as Real,
            };

            assert_eq!(overlapping_or_touching(&p2, &p1), None);
        }
    }
    #[test]
    fn test_project() {
        let points = vec![
            P3::new(ONE, ONE, ZERO),
            P3::new(2 as Real, 2 as Real, ZERO),
            P3::new(3 as Real, 3 as Real, ZERO),
        ];

        let proj = project(&Directions::right(), &points);
        assert_eq!(proj.min, ONE);
        assert_eq!(proj.max, 3 as Real);
    }

    struct Square {
        transform: Transform,
        halfsize: (Real, Real),
    }

    impl Square {
        fn vertices(&self) -> Vec<P3> {
            let position = self.transform.position();
            vec![
                P3::new(
                    position[0] - self.halfsize.0,
                    position[1] - self.halfsize.1,
                    ZERO,
                ),
                P3::new(
                    position[0] + self.halfsize.0,
                    position[1] - self.halfsize.1,
                    ZERO,
                ),
                P3::new(
                    position[0] + self.halfsize.0,
                    position[1] + self.halfsize.1,
                    ZERO,
                ),
                P3::new(
                    position[0] - self.halfsize.0,
                    position[1] + self.halfsize.1,
                    ZERO,
                ),
            ]
        }
    }

    impl SAT for Square {
        fn separating_axis(&self) -> Vec<Vec3> {
            vec![
                self.transform.rotation.row(0),
                self.transform.rotation.row(1),
            ]
        }
    }
    #[test]
    fn separating_axis_2d() {
        // sat touching
        {
            let square1 = Square {
                transform: Transform::identity(),
                halfsize: (0.5, 0.5),
            };
            let square2 = Square {
                transform: Transform::translation(Directions::right()),
                halfsize: (0.5, 0.5),
            };

            let sat = sat_2D(
                &square1.vertices(),
                &square1.separating_axis(),
                &square2.vertices(),
                &square2.separating_axis(),
            );
            assert!(sat.is_some());

            let sat_unwrapped = sat.unwrap();
            let mut distance;
            let mut axis = Vec3::zeros();
            if sat_unwrapped.face_A.distance < sat_unwrapped.face_B.distance {
                distance = sat_unwrapped.face_A.distance;
                axis = sat_unwrapped.face_A.axis;
            } else {
                distance = sat_unwrapped.face_B.distance;
                axis = sat_unwrapped.face_B.axis;
            }

            assert_eq!(distance, ZERO);

            assert_eq!(axis.x(), ONE);
            assert_eq!(axis.y(), ZERO);
            assert_eq!(axis.z(), ZERO);
        }
        // sat overlapping
        {
            let square1 = Square {
                transform: Transform::identity(),
                halfsize: (0.5, 0.5),
            };
            let square2 = Square {
                transform: Transform::translation(Directions::right()),
                halfsize: (0.6, 0.5),
            };

            let sat = sat_2D(
                &square1.vertices(),
                &square1.separating_axis(),
                &square2.vertices(),
                &square2.separating_axis(),
            );
            assert!(sat.is_some());
            let sat_unwrapped = sat.unwrap();
            let mut distance = ZERO;
            let mut axis = Vec3::zeros();
            if sat_unwrapped.face_A.distance < sat_unwrapped.face_B.distance {
                distance = sat_unwrapped.face_A.distance;
                axis = sat_unwrapped.face_A.axis;
            } else {
                distance = sat_unwrapped.face_B.distance;
                axis = sat_unwrapped.face_B.axis;
            }

            assert_approx_eq!(distance, 0.1, 1.0e-6);

            assert_eq!(axis.x(), ONE);
            assert_eq!(axis.y(), ZERO);
            assert_eq!(axis.z(), ZERO);
        }

        // no projection on all axis, no intersection

        {
            let square1 = Square {
                transform: Transform::identity(),
                halfsize: (0.5, 0.5),
            };
            let square2 = Square {
                transform: Transform::translation(Vec3::new(2 as Real, ZERO, ZERO)),
                halfsize: (0.5, 0.5),
            };
            let sat = sat_2D(
                &square1.vertices(),
                &square1.separating_axis(),
                &square2.vertices(),
                &square2.separating_axis(),
            );
            assert!(sat.is_none());
        }
    }

    /**
     * Same tests as in computing intersection between two OBB
     */
    #[test]
    fn separating_axis_3d() {
        // no intersection
        {
            let obb1 = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let obb2 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Directions::right() * (3 as Real)),
            );

            let r = sat_3D(&obb1, &obb2);
            assert!(r.is_none());
        }

        // oriented
        {
            let obb1 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::rotation(Rotation::Z(std::f32::consts::FRAC_PI_4)),
            );
            let obb2 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Directions::right()),
            );
            let r = sat_3D(&obb1, &obb2);

            assert!(r.is_some());
        }

        // vertex touching
        {
            let obb1 = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let obb2 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Vec3::new(ONE, ONE, ONE)),
            );
            let r = sat_3D(&obb1, &obb2);

            assert!(r.is_some());
        }

        // edge touching
        {
            let obb1 = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let obb2 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Vec3::new(ONE, ONE, ZERO)),
            );
            let r = sat_3D(&obb1, &obb2);

            assert!(r.is_some());
        }

        // face touching
        {
            let obb1 = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let obb2 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Directions::right() * (2 as Real)),
            );
            let r = sat_3D(&obb1, &obb2);
            assert!(r.is_some());
        }

        // intersection between an edge of A and an edge of B without either of the vertices of A or B penetrating the other shape (A/B)
        {
            let obb1 = OBB::new(Vec3::new(0.5, 0.5, 0.5), Transform::identity());
            let obb2 = OBB::new(
                Vec3::new(0.5, 0.5, 0.5),
                Transform::new(
                    Vec3::ones(),
                    Rotation::composed(
                        helper::angle_2_rad(-30 as Real),
                        helper::angle_2_rad(35 as Real),
                        helper::angle_2_rad(45 as Real),
                    ),
                    Vec3::new(1 as Real, 0.9 as Real, ZERO),
                ),
            );
            let r = sat_3D(&obb1, &obb2);
            assert!(r.is_some());
        }

        {
            let obb1 = OBB::new(Vec3::new(0.5, 0.5, 0.5), Transform::identity());
            let obb2 = OBB::new(
                Vec3::new(0.5, 0.5, 0.5),
                Transform::new(
                    Vec3::ones(),
                    Rotation::composed(
                        helper::angle_2_rad(30 as Real),
                        helper::angle_2_rad(-35 as Real),
                        helper::angle_2_rad(-45 as Real),
                    ),
                    Vec3::new(1.2 as Real, 1.2 as Real, ZERO),
                ),
            );
            let r = sat_3D(&obb1, &obb2);
            assert!(r.is_none());
        }
    }
    // #[test]
    // fn test_normal() {
    //     let obb1 = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());

    //     let obb2 = OBB::new(
    //         Vec3::new(ONE, ONE, ONE),
    //         Transform::new(
    //             Vec3::ones(),
    //             Rotation::composed(
    //                 helper::angle_2_rad(30 as Real),
    //                 helper::angle_2_rad(-35 as Real),
    //                 helper::angle_2_rad(-45 as Real),
    //             ),
    //             Vec3::new(1.125 as Real, 0.934 as Real, ZERO),
    //         ),
    //     );

    //     for sep in obb2.sat_separating_axis() {
    //         let normal_computed = obb2.computed_face_normal(sep);
    //         println!("id :{:?}", sep);
    //         println!("rotation:{:?} \n", obb2.face_normal(sep));
    //         println!("computed:{:?} \n", normal_computed);
    //     }
    // }
}
