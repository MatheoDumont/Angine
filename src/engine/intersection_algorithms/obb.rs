use crate::engine::shapes::OBB;
use crate::geometry::{sat, sat::SAT};
use crate::math::math_essentials::*;

pub fn obb_obb(obb1: &OBB, obb2: &OBB) -> bool {
    // let p = obb1.closest_point(&P3::from(obb2.transform.translation));
    // obb2.is_inside(&p)
    // obb1 to obb2
    let d = obb1.transform.position() - obb2.transform.position();
    let l = magnitude(&d);
    let n = d / l;

    // Cette méthode fause les distance en allant du centre de obb1 vers celui de obb2, mais est rapide.
    // Voir en pratique si fonctionnelle ou non, sinon utiliser la methode des axes séparateurs
    let distance_obb1_vers_obb2 = obb1.half_side[0]
        * dot(&n, &obb1.transform.rotation.row(0)).abs()
        + obb1.half_side[1] * dot(&n, &obb1.transform.rotation.row(1)).abs()
        + obb1.half_side[2] * dot(&n, &obb1.transform.rotation.row(2)).abs();
    let distance_obb2_vers_obb1 = obb2.half_side[0]
        * dot(&n, &obb2.transform.rotation.row(0)).abs()
        + obb2.half_side[1] * dot(&n, &obb2.transform.rotation.row(1)).abs()
        + obb2.half_side[2] * dot(&n, &obb2.transform.rotation.row(2)).abs();

    // let penetration_distance = l - distance_obb1_vers_obb2 + distance_obb2_vers_obb1;
    // and normal is n, from obb1 to obb2, on peut changer le test de retour comme étant penetration_distance > 0 pour réduire d'un calcul
    distance_obb1_vers_obb2 + distance_obb2_vers_obb1 >= l

    // let r = sat::SeparatingAxisMethod::compute_3D(
    //     &obb1.vertices(),
    //     &obb1.separating_axis(),
    //     &obb2.vertices(),
    //     &obb2.separating_axis(),
    // );

    // r.is_some()
}

#[cfg(test)]
mod tests {
    use super::obb_obb;
    use crate::engine::shapes::OBB;
    use crate::math::math_essentials::*;
    #[test]
    fn obb_obb_intersection() {
        // no intersection
        {
            let obb1 = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let obb2 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Directions::right() * (3 as Real)),
            );
            assert_eq!(obb_obb(&obb1, &obb2), false);
        }
        // intersection on contour
        {
            let obb1 = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let obb2 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Directions::right() * (2 as Real)),
            );
            assert_eq!(obb_obb(&obb1, &obb2), true);
        }
        // big intersect
        {
            let obb1 = OBB::new(Vec3::new(ONE, ONE, ONE), Transform::identity());
            let obb2 = OBB::new(
                Vec3::new(ONE, ONE, ONE),
                Transform::translation(Directions::right()),
            );
            assert_eq!(obb_obb(&obb1, &obb2), true);
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
            assert_eq!(obb_obb(&obb1, &obb2), true);
        }

        // intersection between an edge of A and an edge of B without either of the vertices of A or B penetrating the other shape (A/B)
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
                    Vec3::new(1 as Real, 0.9 as Real, ZERO),
                ),
            );
            assert_eq!(obb_obb(&obb1, &obb2), true);
            assert_eq!(obb_obb(&obb2, &obb1), true);
        }

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
            assert_eq!(obb_obb(&obb1, &obb2), true);
            assert_eq!(obb_obb(&obb2, &obb1), true);
        }
    }
}
