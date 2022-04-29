use super::{Shape, ShapeType};
use crate::geometry::geometry_traits::PolyhedronTrait;
use crate::math::{math_essentials::*, Mat3};

pub struct Plane {
    pub normal: Vec3,
    pub distance_from_origin: Real,
    transform: Transform,
}

impl Plane {
    pub fn new(normal: Vec3) -> Plane {
        #[cfg(debug_assertions)]
        let m = magnitude(&normal);
        let mut normal = normal;
        if m != ONE {
            normal /= m;
        }

        Plane {
            normal: normal,
            distance_from_origin: ZERO,
            transform: Transform::identity(),
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
        let mut p = Plane::new(polyhedron.face_normal(face_idx));
        p.set_position(P3::new(
            midpoint[0] / size,
            midpoint[1] / size,
            midpoint[2] / size,
        ));
        p
    }

    /**
     * Positive distance if p is in the direction of the normal  
     * Negative distance if p is in the opposite direction  
     * 0 if on
     */
    pub fn signed_distance(&self, p: &P3) -> Real {
        let ap = p - &self.transform.position();
        dot(&self.normal, &ap)
    }

    /**
     * Reject le point sur le plan
     */
    pub fn reject_point_on_plane(&self, p: &P3) -> Vec3 {
        // rejection en prennant en compte la distance au plan
        rejection(&self.normal, &p) + projection(&self.normal, self.get_position())
    }

    fn compute_distance_from_origin(&mut self) {
        self.distance_from_origin = magnitude(&self.transform.translation);
    }
}

impl Shape for Plane {
    fn compute_inertia_matrix(&self, mass: Real) -> Mat3 {
        // pour l'instant
        panic!("inertia matrix for Plane not implemented");
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
        self.compute_distance_from_origin();
    }
    fn set_orientation(&mut self, o: Mat3) {
        self.transform.rotation = o;
    }

    fn get_transform(&self) -> Transform {
        self.transform
    }
    fn set_transform(&mut self, t: Transform) {
        self.transform = t;
        self.compute_distance_from_origin();
    }
}

#[cfg(test)]
mod tests {
    use super::Plane;
    use crate::engine::shapes::Shape;
    use crate::math::{math_essentials::*, Mat3};
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn plane_signed_distance() {
        {
            let mut plan = Plane::new(Directions::up());
            plan.transform.translation = P3::new(ZERO, ONE, ZERO);
            let p = P3::new(ZERO, 2 as Real, ZERO);

            assert_approx_eq!(plan.signed_distance(&p), ONE, 1.0e-6);
        }

        {
            let mut plan = Plane::new(Directions::up());
            plan.transform.translation = P3::new(ZERO, ONE, ZERO);
            let p = P3::new(ZERO, -ONE, ZERO);

            assert_approx_eq!(plan.signed_distance(&p), -2 as Real, 1.0e-6);
        }

        {
            let plan = Plane::new(Directions::up());
            let p = P3::new(1.5, 1.0, 1.0);

            assert_approx_eq!(plan.signed_distance(&p), dot(&p, &plan.normal), 1.0e-6);
        }

        {
            let plan = Plane::new(Rotation::Y(std::f32::consts::FRAC_PI_4) * Directions::right());
            let p = P3::new(ZERO, ZERO, -ONE);
            assert_approx_eq!(plan.signed_distance(&p), dot(&p, &plan.normal), 1.0e-6);
        }

        {
            let plan = Plane::new(Directions::up());
            let p = P3::from(Directions::up());

            assert_approx_eq!(plan.signed_distance(&p), ONE, 1.0e-6);
        }

        {
            let plan = Plane::new(Directions::up());
            let p = P3::origin();

            assert_approx_eq!(plan.signed_distance(&p), ZERO, 1.0e-6);
        }
    }

    #[test]
    fn test_reject_point_on_plane() {
        {
            let p = P3::new(1.0, 2.0, 3.0);
            let plane = Plane::new(Directions::up());

            let r = plane.reject_point_on_plane(&p);
            assert_eq!(r.x(), 1.0);
            assert_eq!(r.y(), 0.0);
            assert_eq!(r.z(), 3.0);
        }

        {
            let p = P3::new(1.0, 2.0, 3.0);
            let plane = Plane::new(Directions::right());

            let r = plane.reject_point_on_plane(&p);
            assert_eq!(r.x(), 0.0);
            assert_eq!(r.y(), 2.0);
            assert_eq!(r.z(), 3.0);
        }

        {
            let p = P3::new(1.0, 2.0, 3.0);
            let mut plane = Plane::new(Directions::right());
            plane.set_transform(Transform::translation(Directions::right() * -3.0));

            let r = plane.reject_point_on_plane(&p);
            assert_eq!(r.x(), -3.0);
            assert_eq!(r.y(), 2.0);
            assert_eq!(r.z(), 3.0);
        }

        {
            let p = P3::new(1.0, 2.0, 3.0);
            let mut plane = Plane::new(-Directions::up());
            plane.set_transform(Transform::translation(Directions::up() * 3.0));

            let r = plane.reject_point_on_plane(&p);
            assert_eq!(r.x(), 1.0);
            assert_eq!(r.y(), 3.0);
            assert_eq!(r.z(), 3.0);
        }
    }
}
