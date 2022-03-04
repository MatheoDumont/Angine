/**
 * Implementation of the Separating Axis Theorem
 */
use super::{geometry_traits::*, Real, Vec3, P3, ZERO};

pub trait SAT {
    fn separating_axis(&self) -> Vec<Vec3>;
}

/**
 * Compute the cross product between each axis of both shapes.
 */
pub fn cross_separating_axis(axis1: &Vec<Vec3>, axis2: &Vec<Vec3>) -> Vec<Vec3> {
    let mut v = Vec::with_capacity(axis1.len() * axis2.len());

    for shape_axis1 in axis1 {
        for shape_axis2 in axis2 {
            let mut cross = shape_axis1.cross(shape_axis2);
            let norm = cross.norm();
            if norm != ZERO {
                cross /= norm;
                v.push(cross);
            }
        }
    }
    v
}

pub struct Projection {
    min: Real,
    max: Real,
}

pub struct SeparatingAxisMethod {
    pub minimum_translation: Real,
    pub minimum_axis: Vec3,
}

impl SeparatingAxisMethod {
    pub fn compute_2D(
        shape1_vertices: &Vec<P3>,
        axis_shape1: &Vec<Vec3>,
        shape2_vertices: &Vec<P3>,
        axis_shape2: &Vec<Vec3>,
    ) -> Option<SeparatingAxisMethod> {
        let mut minimum_translation = f32::MAX;
        let mut minimum_axis = &Vec3::zero();

        for axis in axis_shape1 {
            let proj1 = project(axis, &shape1_vertices);
            let proj2 = project(axis, &shape2_vertices);

            if let Some(x) = overlapping_or_touching(&proj1, &proj2) {
                if x < minimum_translation {
                    minimum_translation = x;
                    minimum_axis = axis;
                }
            } else {
                return None;
            }
        }

        for axis in axis_shape2 {
            let proj1 = project(axis, &shape1_vertices);
            let proj2 = project(axis, &shape2_vertices);

            if let Some(x) = overlapping_or_touching(&proj1, &proj2) {
                if x < minimum_translation {
                    minimum_translation = x;
                    minimum_axis = axis;
                }
            } else {
                return None;
            }
        }

        Some(SeparatingAxisMethod {
            minimum_translation: minimum_translation,
            minimum_axis: minimum_axis.clone(),
        })
    }

    pub fn compute_3D(
        shape1_vertices: &Vec<P3>,
        axis_shape1: &Vec<Vec3>,
        shape2_vertices: &Vec<P3>,
        axis_shape2: &Vec<Vec3>,
    ) -> Option<SeparatingAxisMethod> {
        let mut minimum_translation = f32::MAX;
        let mut minimum_axis = &Vec3::zero();

        for axis in axis_shape1 {
            let proj1 = project(axis, &shape1_vertices);
            let proj2 = project(axis, &shape2_vertices);

            if let Some(x) = overlapping_or_touching(&proj1, &proj2) {
                if x < minimum_translation {
                    minimum_translation = x;
                    minimum_axis = axis;
                }
            } else {
                return None;
            }
        }

        for axis in axis_shape2 {
            let proj1 = project(axis, &shape1_vertices);
            let proj2 = project(axis, &shape2_vertices);

            if let Some(x) = overlapping_or_touching(&proj1, &proj2) {
                if x < minimum_translation {
                    minimum_translation = x;
                    minimum_axis = axis;
                }
            } else {
                return None;
            }
        }

        let cross_axis = cross_separating_axis(axis_shape1, axis_shape2);
        for axis in &cross_axis {
            let proj1 = project(axis, &shape1_vertices);
            let proj2 = project(axis, &shape2_vertices);

            if let Some(x) = overlapping_or_touching(&proj1, &proj2) {
                if x < minimum_translation {
                    minimum_translation = x;
                    minimum_axis = axis;
                }
            } else {
                return None;
            }
        }

        Some(SeparatingAxisMethod {
            minimum_translation: minimum_translation,
            minimum_axis: minimum_axis.clone(),
        })
    }
}

fn project(axis: &Vec3, vertices: &Vec<P3>) -> Projection {
    let mut min = axis.dot_point(&vertices[0]);
    let mut max = min;

    for i in 1..vertices.len() {
        let x = axis.dot_point(&vertices[i]);
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

        let proj = project(&Vec3::right(), &points);
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
                transform: Transform::translation(Vec3::right()),
                halfsize: (0.5, 0.5),
            };

            let sat = SeparatingAxisMethod::compute_2D(
                &square1.vertices(),
                &square1.separating_axis(),
                &square2.vertices(),
                &square2.separating_axis(),
            );
            assert!(sat.is_some());
            let sat_unwrapped = sat.unwrap();

            assert_eq!(sat_unwrapped.minimum_translation, ZERO);

            let axis = sat_unwrapped.minimum_axis;
            assert_eq!(axis.x, ONE);
            assert_eq!(axis.y, ZERO);
            assert_eq!(axis.z, ZERO);
        }
        // sat overlapping
        {
            let square1 = Square {
                transform: Transform::identity(),
                halfsize: (0.5, 0.5),
            };
            let square2 = Square {
                transform: Transform::translation(Vec3::right()),
                halfsize: (0.6, 0.5),
            };

            let sat = SeparatingAxisMethod::compute_2D(
                &square1.vertices(),
                &square1.separating_axis(),
                &square2.vertices(),
                &square2.separating_axis(),
            );
            assert!(sat.is_some());
            let sat_unwrapped = sat.unwrap();

            assert_approx_eq!(sat_unwrapped.minimum_translation, 0.1, 1.0e-6);

            let axis = sat_unwrapped.minimum_axis;
            assert_eq!(axis.x, ONE);
            assert_eq!(axis.y, ZERO);
            assert_eq!(axis.z, ZERO);
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
            let sat = SeparatingAxisMethod::compute_2D(
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
                Transform::translation(Vec3::right() * (3 as Real)),
            );

            let r = SeparatingAxisMethod::compute_3D(
                &obb1.vertices(),
                &obb1.separating_axis(),
                &obb2.vertices(),
                &obb2.separating_axis(),
            );

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
                Transform::translation(Vec3::right()),
            );
            let r = SeparatingAxisMethod::compute_3D(
                &obb1.vertices(),
                &obb1.separating_axis(),
                &obb2.vertices(),
                &obb2.separating_axis(),
            );

            assert!(r.is_some());
        }

        // vertex touching
        {
            let obb1 = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let obb2 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Vec3::new(ONE, ONE, ONE)),
            );
            let r = SeparatingAxisMethod::compute_3D(
                &obb1.vertices(),
                &obb1.separating_axis(),
                &obb2.vertices(),
                &obb2.separating_axis(),
            );

            assert!(r.is_some());
        }

        // edge touching
        {
            let obb1 = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let obb2 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Vec3::new(ONE, ONE, ZERO)),
            );
            let r = SeparatingAxisMethod::compute_3D(
                &obb1.vertices(),
                &obb1.separating_axis(),
                &obb2.vertices(),
                &obb2.separating_axis(),
            );

            assert!(r.is_some());
        }

        // face touching
        {
            let obb1 = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let obb2 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Vec3::right() * (2 as Real)),
            );
            let r = SeparatingAxisMethod::compute_3D(
                &obb1.vertices(),
                &obb1.separating_axis(),
                &obb2.vertices(),
                &obb2.separating_axis(),
            );

            assert!(r.is_some());
        }

        // intersection between an edge of A and an edge of B without either of the vertices of A or B penetrating the other shape (A/B)
        {
            let obb1 = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let obb2 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::new(
                    Vec3::ones(),
                    Rotation::composed(
                        helper::angle_2_rad(30 as Real),
                        helper::angle_2_rad(-35 as Real),
                        helper::angle_2_rad(-45 as Real),
                    ),
                    Vec3::new(2.1 as Real, 1.7 as Real, ZERO),
                ),
            );
            let r = SeparatingAxisMethod::compute_3D(
                &obb1.vertices(),
                &obb1.separating_axis(),
                &obb2.vertices(),
                &obb2.separating_axis(),
            );

            assert!(r.is_some());
        }

        // almost the same but no intersection
        {
            let obb1 = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let obb2 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::new(
                    Vec3::ones(),
                    Rotation::composed(
                        helper::angle_2_rad(30 as Real),
                        helper::angle_2_rad(-35 as Real),
                        helper::angle_2_rad(-45 as Real),
                    ),
                    Vec3::new(2.1 as Real, 2.5 as Real, ZERO),
                ),
            );
            let r = SeparatingAxisMethod::compute_3D(
                &obb1.vertices(),
                &obb1.separating_axis(),
                &obb2.vertices(),
                &obb2.separating_axis(),
            );

            assert!(r.is_none());
        }
    }
}
