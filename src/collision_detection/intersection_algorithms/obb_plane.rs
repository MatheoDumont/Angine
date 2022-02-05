use crate::collision_detection::shapes::{Plane, OBB};
use crate::math::{Vec3, P3, ZERO};

pub fn obb_plane(obb: &OBB, plane: &Plane) -> bool {
    let d = Vec3::new(
        plane.normal.x * obb.half_side.x,
        plane.normal.y * obb.half_side.y,
        plane.normal.z * obb.half_side.z,
    );
    let dot_p_contour = (obb.transform.translation - d).dot(&plane.normal);
    let position_obb = (obb.transform.translation).dot(&plane.normal);

    // si un point est positif et l'autre négatif alors ils sont de part et d'autre du plan
    // donc il y a intersection
    if dot_p_contour >= ZERO && position_obb <= ZERO
        || dot_p_contour <= ZERO && position_obb >= ZERO
    {
        true
    } else {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::obb_plane;
    use crate::collision_detection::shapes::{Plane, OBB};
    use crate::math::{Real, Rotation, Transform, Vec3, ONE, P3, ZERO};
    #[test]
    fn obb_sphere_intersection() {
        {
            let obb = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let plane = Plane::new(Vec3::up(), position: P3)
        }
    }
}
