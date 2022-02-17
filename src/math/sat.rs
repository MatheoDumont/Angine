/**
 * Implementation of the Separating Axis Theorem
 */
use super::{traits::Mesh, Real, Vec3, P3};

pub trait SAT {
    fn separating_axis(&self) -> Vec<Vec3>;
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
    pub fn compute<T: SAT + Mesh>(shape1: &T, shape2: &T) -> Option<SeparatingAxisMethod> {
        let axis_shape1 = shape1.separating_axis();
        let axis_shape2 = shape2.separating_axis();
        let shape1_vertices = shape1.vertices();
        let shape2_vertices = shape2.vertices();

        let mut minimum_translation = f32::MAX;
        let mut minimum_axis = &Vec3::zero();

        for axis in &axis_shape1 {
            let proj1 = project(axis, &shape1_vertices);
            let proj2 = project(axis, &shape2_vertices);

            if let Some(x) = overlap(&proj1, &proj2) {
                if x < minimum_translation {
                    minimum_translation = x;
                    minimum_axis = axis;
                }
            } else {
                return None;
            }
        }

        for axis in &axis_shape2 {
            let proj1 = project(axis, &shape1_vertices);
            let proj2 = project(axis, &shape2_vertices);

            if let Some(x) = overlap(&proj1, &proj2) {
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

fn overlap(p1: &Projection, p2: &Projection) -> Option<Real> {
    if p1.max > p2.min && p2.max > p1.min {
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

    #[test]
    fn test_overlap() {
        {
            let p1 = Projection {
                min: 1 as Real,
                max: 3 as Real,
            };
            let p2 = Projection {
                min: 2.5 as Real,
                max: 4 as Real,
            };

            assert_eq!(overlap(&p1, &p2), Some(0.5));
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

            assert_eq!(overlap(&p2, &p1), Some(0.5));
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

            assert_eq!(overlap(&p2, &p1), Some(0.5));
        }
    }
}
