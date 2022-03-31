use crate::math::math_essentials::*;

pub struct Line {
    pub a: P3,
    pub b: P3,
}

impl Line {
    // pub fn closest_points(&self) -> [P3; 2] {
    //     []
    // }

    /**
     * source:
     * http://math.15873.pagesperso-orange.fr/page10.htm
     * Pour que 2 lignes s'intersectent, elles doivent etre coplanaire (cad sur le meme plan)
     * et non parallèle.
     * On ne test ni la propriété coplanaire ni parallèle => les deux droites sont supposés etre coplanaire et non parallèle.
     *
     * elles sont coplanaire si elles sont parallèles ou sécantes (https://fr.wikipedia.org/wiki/Coplanaire)
     * elles sont parallele si cross(self.b - self.a, o.b - o.a) = 0
     * elles sont sécantes si (b1 - b2)*(k1 - k2) == (d1 - d2)*(a1 - a2) (http://math.15873.pagesperso-orange.fr/page10.htm)
     */
    fn intersecting_point(&self, other_line: &Line) -> P3 {
        // 3 cas spéciaux : si les droites sont sur un plan totalement perpendiculaire à un des axe x, y ou z
        // et donc 1 cas général: si le plan est non aligné
        let r1 = self.b - self.a;
        let r2 = other_line.b - other_line.a;
        let mut r1_p = r1.clone();
        let mut r2_p = r2.clone();

        if r1_p.x() == ZERO {
            r1_p[0] = ONE;
        }
        if r1_p.y() == ZERO {
            r1_p[1] = ONE;
        }
        if r1_p.z() == ZERO {
            r1_p[2] = ONE;
        }
        if r2_p.x() == ZERO {
            r2_p[0] = ONE;
        }
        if r2_p.y() == ZERO {
            r2_p[1] = ONE;
        }
        if r2_p.z() == ZERO {
            r2_p[2] = ONE;
        }

        let mut p = self.a.clone();
        // perpendicular to X axis
        if dot(&r1, &Directions::right()) == ZERO {
            // y = az+b
            let a1 = r1_p.y() / r1_p.z();
            let b1 = dot(&self.a, &Directions::up());
            let a2 = r2_p.y() / r2_p.z();
            let b2 = dot(&other_line.a, &Directions::up());

            p[2] = (b2 - b1) / (a1 - a2);

            // p[1] = (a1 * b2 - a2 * b1) / (a1 - a2);
            p[1] = a1 * p.z() + b1;
        }
        // perpendicular to Y axis
        else if dot(&r1, &Directions::up()) == ZERO {
            // z = ax+b
            let a1 = r1_p.z() / r1_p.x();
            let b1 = dot(&self.a, &Directions::forward());
            let a2 = r2_p.z() / r2_p.x();
            let b2 = dot(&other_line.a, &Directions::forward());
            p[0] = (b2 - b1) / (a1 - a2);
            // p[2] = (a1 * b2 - a2 * b1) / (a1 - a2);
            p[2] = a1 * p.x() + b1;
        }
        // perpendicular to Z axis
        else if dot(&r1, &Directions::forward()) == ZERO {
            // y = ax+b
            let a1 = r1_p.y() / r1_p.x();
            let b1 = dot(&self.a, &Directions::up());
            let a2 = r2_p.y() / r2_p.x();
            let b2 = dot(&other_line.a, &Directions::up());

            p[0] = (b2 - b1) / (a1 - a2);
            // p[1] = (a1 * b2 - a2 * b1) / (a1 - a2);
            p[1] = a1 * p.x() + b1;
        }
        // perpendicular to general case
        else {
            // y = ax+b
            let a1 = r1_p.y() / r1_p.x();
            let b1 = dot(&self.a, &Directions::up());
            let a2 = r2_p.y() / r2_p.x();
            let b2 = dot(&other_line.a, &Directions::up());

            // z = kx+d
            let k1 = r1_p.z() / r1_p.x();
            let d1 = dot(&self.a, &Directions::forward());
            // let k2 = r2_p.z() / r2_p.x();
            // let d2 = dot(&other_line.a, &Directions::forward());

            p[0] = (b2 - b1) / (a1 - a2);
            p[1] = a1 * p.x() + b1;
            p[2] = k1 * p.x() + d1;
        }

        p
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq;
    #[test]
    fn test_intersecting_point() {
        // perpendicular to X axis
        {
            let l1 = Line {
                a: P3::new(3.0, 0.0, 0.0),
                b: P3::new(3.0, 1.0, 1.0),
            };
            let l2 = Line {
                a: P3::new(3.0, 3.0, 0.0),
                b: P3::new(3.0, 0.0, 1.5),
            };

            let r = l1.intersecting_point(&l2);

            assert_approx_eq!(r.x(), 3.0);
            assert_approx_eq!(r.y(), 1.0);
            assert_approx_eq!(r.z(), 1.0);
        }

        // perpendicular to Y axis
        {
            let l1 = Line {
                a: P3::new(ZERO, 3.0, ZERO),
                b: P3::new(ONE, 3.0, ONE),
            };
            let l2 = Line {
                a: P3::new(ZERO, 3.0, 3.0),
                b: P3::new(1.5, 3.0, ZERO),
            };

            let r = l1.intersecting_point(&l2);

            assert_approx_eq!(r.x(), 1.0);
            assert_approx_eq!(r.y(), 3.0);
            assert_approx_eq!(r.z(), 1.0);
        }

        // perpendicular to Z axis
        {
            let l1 = Line {
                a: P3::origin(),
                b: P3::new(1.0, 1.0, 0.0),
            };
            let l2 = Line {
                a: P3::new(0.0, -1.0, 0.0),
                b: P3::new(1.0, 2.0, 0.0),
            };

            let r = l1.intersecting_point(&l2);

            assert_approx_eq!(r.x(), 0.5);
            assert_approx_eq!(r.y(), 0.5);
            assert_approx_eq!(r.z(), 0.0);
        }

        // general case, rotate perpendicular plan Z axis  for 45° around y axis
        {
            let rotation = Rotation::Y(helper::angle_2_rad(45.0));
            let l1 = Line {
                a: P3::origin(),
                b: rotation * P3::new(1.0, 1.0, 0.0),
            };
            let l2 = Line {
                a: rotation * P3::new(0.0, -1.0, 0.0),
                b: rotation * P3::new(1.0, 2.0, 0.0),
            };

            let r = l1.intersecting_point(&l2);
            let intersect_point = P3::new(0.5, 0.5, ZERO);
            let rotated_intersect_point = rotation * intersect_point;
            assert_approx_eq!(r.x(), rotated_intersect_point.x());
            assert_approx_eq!(r.y(), rotated_intersect_point.y());
            assert_approx_eq!(r.z(), rotated_intersect_point.z());
        }
    }
}
