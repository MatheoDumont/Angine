use super::{Shape, ShapeType};
use crate::geometry::geometry_traits::PolyhedronTrait;
use crate::math::{math_essentials::*, Mat3};

pub struct Plane {
    pub normal: Vec3,
    pub distance_from_origin: Real,
    pub transform: Transform,
    pub inertia_matrix: Mat3,
    pub inv_inertia_matrix: Mat3,
}

impl Plane {
    pub fn new(normal: Vec3, position: P3) -> Plane {
        #[cfg(debug_assertions)]
        let m = magnitude(&normal);
        let mut normal = normal;
        if m != ONE {
            normal /= m;
        }

        Plane {
            normal: normal,
            distance_from_origin: magnitude(&position),
            transform: Transform::translation(position),
            inertia_matrix: Mat3::identity(),
            inv_inertia_matrix: Mat3::identity(),
        }
    }

    pub fn from_polyhedron_face<T: PolyhedronTrait>(polyhedron: &T, face_idx: usize) -> Plane {
        // compute center point of face
        let face = &polyhedron.faces_ref()[face_idx];
        let mut midpoint = Vec3::zeros();

        for vertex_idx in &face.v_i {
            let p = polyhedron.transformed_vertex(*vertex_idx);
            midpoint[0] += p[0];
            midpoint[1] += p[1];
            midpoint[2] += p[2];
        }
        let size = face.v_i.len() as Real;
        Plane::new(
            polyhedron.face_normal(face_idx),
            P3::new(midpoint[0] / size, midpoint[1] / size, midpoint[2] / size),
        )
    }

    /**
     * Positive distance if p is in the direction of the normal  
     * Negative distance if p is in the opposite direction  
     * 0 if on
     */
    pub fn signed_distance(&self, v: &Vector3) -> Real {
        dot(&self.normal, &v) - self.distance_from_origin
    }

    pub fn point_to_plane(&self, v: &P3) -> Vec3 {
        // rejection en prennant en compte la distance au plan
        -self.normal * (dot(&self.normal, v) - self.distance_from_origin)
    }
}

impl Shape for Plane {
    fn compute_inertia_matrix(&mut self, mass: Real) {
        // pour l'instant
        panic!("inertia matrix for Plane not implemented");
    }

    fn inertia_matrix(&self) -> &Mat3 {
        &self.inertia_matrix
    }
    fn inverse_inertia_matrix(&self) -> &Mat3 {
        &self.inv_inertia_matrix
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Plane
    }

    fn is_rigid_body(&self) -> bool {
        false
    }

    fn get_position(&self) -> &P3 {
        &self.transform.translation
    }
    fn get_orientation(&self) -> &Mat3 {
        &self.transform.rotation
    }
    fn set_position(&mut self, p: P3) {
        self.transform.translation = p;
    }
    fn set_orientation(&mut self, o: Mat3) {
        self.transform.rotation = o;
    }
}

#[cfg(test)]
mod tests {
    use super::Plane;
    use crate::math::{math_essentials::*, Mat3};
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn plane_signed_distance() {
        {
            let plan = Plane::new(Directions::up(), P3::new(ZERO, ONE, ZERO));
            let p = P3::new(ZERO, 2 as Real, ZERO);

            assert_approx_eq!(plan.signed_distance(&p), ONE, 1.0e-6);
        }

        {
            let plan = Plane::new(Directions::up(), P3::new(ZERO, ONE, ZERO));
            let p = P3::new(ZERO, -ONE, ZERO);

            assert_approx_eq!(plan.signed_distance(&p), -2 as Real, 1.0e-6);
        }

        {
            let plan = Plane::new(Directions::up(), P3::origin());
            let p = P3::new(1.5, 1.0, 1.0);

            assert_approx_eq!(plan.signed_distance(&p), dot(&p, &plan.normal), 1.0e-6);
        }

        {
            let plan = Plane::new(
                Rotation::Y(std::f32::consts::FRAC_PI_4) * Directions::right(),
                // P3::new(ZERO, ZERO, ONE),
                P3::origin(),
            );
            let p = P3::new(ZERO, ZERO, -ONE);
            println!("{:?}", plan.normal);
            assert_approx_eq!(plan.signed_distance(&p), dot(&p, &plan.normal), 1.0e-6);
        }

        {
            let plan = Plane::new(Directions::up(), P3::origin());
            let p = P3::from(Directions::up());

            assert_approx_eq!(plan.signed_distance(&p), ONE, 1.0e-6);
        }

        {
            let plan = Plane::new(Directions::up(), P3::origin());
            let p = P3::origin();

            assert_approx_eq!(plan.signed_distance(&p), ZERO, 1.0e-6);
        }
    }
}
