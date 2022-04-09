use crate::math::math_essentials::*;

pub struct Line {
    pub a: P3,
    pub b: P3,
}

impl Line {
    pub fn new(a: P3, b: P3) -> Line {
        Line { a, b }
    }
    pub fn point_on_line(&self, p: P3) -> bool {
        let ap = normalized(&(p - self.a));
        let ab = normalized(&(self.b - self.a));
        let d_rounded = helper::round_n_decimal(dot(&ap, &ab), 6);
        d_rounded.abs() == ONE
    }

    /**
     * Retourne le point sur chaque droite le plus proche de l'autre, donc 2 points en tout.
     * Si les 2 points sont les mêmes, alors les 2 droites s'intersectent.
     * C'est ce que fait la fonction intersect().
     *
     * source: https://math.stackexchange.com/questions/846054/closest-points-on-two-line-segments/883210#883210
     *
     * On définit les droites comme :
     * L1(s) = p1 + s(p2 - p1)
     * L2(t) = p3 + t(p4 - p3)
     *
     * et on pose les équations :
     * (1) (L1(s) - L2(t)) . (p2 - p1) = 0
     * (2) (L1(s) - L2(t)) . (p4 - p3) = 0
     *
     * qui signifient que la droite avec comme points directeurs L1(s) et L2(t) doit être orthogonale aux vecteurs directions
     * de chacune des droites.
     * Sachant qu'une droite orthogonale au vecteurs directions est le plus cours chemin,
     * on obtiens donc les points les plus proches.
     *
     * On développe alors les équations pour obtenir alors ( '.' <=> produit scalaire) :
     * (1') (p2 - p1).(p1 - p3) + s(p2 - p1).(p2 - p1) - t(p2 - p1).(p4 - p3) = 0
     * (2') (p4 - p3).(p1 - p3) + s(p2 - p1).(p4 - p3) - t(p4 - p3).(p4 - p3) = 0
     *
     * en arrangeant l'équation (2') en fonction de s, on obtiens :
     * (3) s = t(p4 - p3).(p4 - p3) - (p4 - p3).(p1 - p3) / (p2 - p1).(p4 - p3)
     *
     * et en remplacant s dans (1') par son équivalence de (3), on obtiens :
     *     (p4 - p3).(p1 - p3) (p2 - p1).(p2 - p1) - (p2 - p1).(p1 - p3) (p2 - p1).(p4 - p3)
     * t = ---------------------------------------------------------------------------------
     *      (p2 - p1).(p2 - p1)(p4 - p3).(p4 - p3) - (p2 - p1).(p4 - p3)(p2 - p1).(p4 - p3)
     *
     * En utilisant le t calculé, nous pouvons calculer s avec (3).
     *
     * Pour le cas où s ou t ne sont pas dans l'intervale  [0, 1]
     * ce ne sont plus les plus proches points, alors on calcule le plus proche point sur une droite par rapport
     * à l'extrémité de l'autre droite :
     * pour L1 et p3 (L1(s) - p3) . (p2 - p1) = 0
     * pour L1 et p4 (L1(s) - p4) . (p2 - p1) = 0
     * pour L2 et p1 (L2(t) - p1) . (p4 - p3) = 0
     * pour L2 et p2 (L2(t) - p2) . (p4 - p3) = 0
     *
     * Puis on choisit pour chaque droite L1 et L2 le point résultant de s ou t minimisant
     * la distance au point de l'extrémité de l'autre droite.
     */
    pub fn closest_point_each_other(&self, line: &Line) -> [P3; 2] {
        let p1 = self.a;
        let p2 = self.b;
        let p3 = line.a;
        let p4 = line.b;

        let d12 = p2 - p1;
        let d34 = p4 - p3;
        let d31 = p1 - p3;
        let dot12 = dot(&d12, &d12);
        let dot34 = dot(&d34, &d34);
        let dot1234 = dot(&d12, &d34);
        let dot3431 = dot(&d34, &d31);

        let t = (dot3431 * dot12 - dot(&d12, &d31) * dot1234) / (dot12 * dot34 - dot1234 * dot1234);
        let s = (t * dot34 - dot3431) / dot1234;

        if s >= ZERO && s <= ONE && t >= ZERO && t <= ONE {
            [
                self.a * (ONE - s) + self.b * s,
                line.a * (ONE - t) + line.b * t,
            ]
        } else {
            /*
             * On calcule 4 points :
             * - sur self le point le plus proche de p3
             *            le point le plus proche de p4
             * - sur line le point le plus proche de p1
             *            le point le plus proche de p2
             *
             * Puis on détermine sur chaque droite, lequel des deux points
             * est le plus proche sur l'autre droite.
             */
            let self_s_p3 = helper::clamp(-dot(&d31, &d12) / dot12, ZERO, ONE);
            let self_s_p4 = helper::clamp(-dot(&(p1 - p4), &d12) / dot12, ZERO, ONE);
            let line_t_p1 = helper::clamp(dot(&d31, &d34) / dot34, ZERO, ONE);
            let line_t_p2 = helper::clamp(-dot(&(p3 - p2), &d34) / dot34, ZERO, ONE);

            // points on self
            let a = self.a * (ONE - self_s_p3) + self.b * self_s_p3;
            let b = self.a * (ONE - self_s_p4) + self.b * self_s_p4;
            let p_on_self;
            if squared_magnitude(&(a - p3)) < squared_magnitude(&(b - p4)) {
                p_on_self = a;
            } else {
                p_on_self = b;
            }

            // points on line
            let c = line.a * (ONE - line_t_p1) + line.b * line_t_p1;
            let d = line.a * (ONE - line_t_p2) + line.b * line_t_p2;
            let p_on_line;
            if squared_magnitude(&(c - p1)) < squared_magnitude(&(d - p2)) {
                p_on_line = c;
            } else {
                p_on_line = d;
            }

            [p_on_self, p_on_line]
        }
    }
    pub fn intersect(&self, line: &Line) -> Option<P3> {
        let pts = self.closest_point_each_other(line);

        if helper::same_points(&pts[0], &pts[1]) {
            Some(pts[0])
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq;
    #[test]
    fn test_closest_point_on_lines() {
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

            let r = l1.intersect(&l2);
            assert!(r.is_some());

            let r = r.unwrap();

            assert_eq!(l1.point_on_line(r), true);
            assert_eq!(l2.point_on_line(r), true);

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

            let r = l1.intersect(&l2);
            assert!(r.is_some());

            let r = r.unwrap();

            assert_eq!(l1.point_on_line(r), true);
            assert_eq!(l2.point_on_line(r), true);
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

            let r = l1.intersect(&l2);
            assert!(r.is_some());

            let r = r.unwrap();

            assert_eq!(l1.point_on_line(r), true);
            assert_eq!(l2.point_on_line(r), true);
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

            let r = l1.intersect(&l2);
            assert!(r.is_some());

            let r = r.unwrap();

            assert_eq!(l1.point_on_line(r), true);
            assert_eq!(l2.point_on_line(r), true);

            let intersect_point = P3::new(0.5, 0.5, ZERO);
            let rotated_intersect_point = rotation * intersect_point;
            assert_approx_eq!(r.x(), rotated_intersect_point.x());
            assert_approx_eq!(r.y(), rotated_intersect_point.y());
            assert_approx_eq!(r.z(), rotated_intersect_point.z());
        }
        // not intersecting
        {
            let l1 = Line {
                a: P3::origin(),
                b: Directions::up() * 5.0,
            };
            let l2 = Line {
                a: P3::new(1.0, 1.0, -3.0),
                b: P3::new(1.0, 1.0, 3.0),
            };
            let r = l1.intersect(&l2);
            assert!(r.is_none());
        }

        // not intersecting closest point
        {
            let l1 = Line {
                a: P3::origin(),
                b: Directions::up() * 5.0,
            };
            let l2 = Line {
                a: P3::new(1.0, 1.0, -3.0),
                b: P3::new(1.0, 1.0, 3.0),
            };
            let r = l1.closest_point_each_other(&l2);
            assert_approx_eq!(r[0].x(), ZERO);
            assert_approx_eq!(r[0].y(), ONE);
            assert_approx_eq!(r[0].z(), ZERO);
        }
    }
}
