use crate::math::math_essentials::*;

pub fn closest_to_p(p: &P3, pts: &Vec<P3>) -> usize {
    let mut closest_vertex = 0;
    let mut distance = squared_magnitude(&(&pts[closest_vertex] - p));
    for i in 1..pts.len() {
        let dd = squared_magnitude(&(p - &pts[i]));
        if distance > dd {
            distance = dd;
            closest_vertex = i;
        }
    }
    closest_vertex
}

pub fn closest_projection(normal: &Vec3, points: &Vec<P3>) -> (Real, usize) {
    let mut smallest_distance = f32::MAX;
    let mut closest_point_index = 0;
    for i in 0..points.len() {
        let x = dot(&normal, &points[i]);
        if x < smallest_distance {
            smallest_distance = x;
            closest_point_index = i;
        }
    }

    (smallest_distance, closest_point_index)
}

pub fn mean_point(points: &Vec<P3>) -> P3 {
    let mut midpoint = P3::origin();

    for p in points {
        midpoint += *p;
    }
    let size = points.len() as Real;
    midpoint / size
}

/**
 * pour tous les points de adjacent_face_index ou dot(&p, side_normal) > 0, ramener pour dot(&p, side_normal) == 0
 * ou p est un des points composant la face adjacent_face_index
 */
pub fn clip(vertices_to_clip: &mut Vec<P3>, clipping_normal: Vec3, vertex_on_face: &Vec3) {
    for v in vertices_to_clip.iter_mut() {
        let face2vertex = *v - *vertex_on_face;

        if dot(&clipping_normal, &v) > ZERO {
            *v -= projection(&clipping_normal, &face2vertex);
        }
    }
}

pub fn face_normal(p1: &P3, p2: &P3, p3: &P3) -> Vec3 {
    let d1 = p3 - p2;
    let d2 = p1 - p2;
    let mut normal = cross(&d1, &d2);
    normalize(&mut normal);

    // reoriente la normale pour partir de A
    if dot(&normal, &p1) < ZERO {
        normal = -normal;
    }

    normal
}
/**
 * Return a perpendicular plan to v.
 * v is not normalized in the function.
 * If v is normalized, then we have an orthonomal basis. 
 */
pub fn perp(v: &Vec3) -> (Vec3, Vec3) {
    let mut v1 = cross(&v, &Directions::up());
    // v1 and y axis are parallele
    if v1 == Vec3::zeros() {
        v1 = cross(&v, &Directions::right());
    }

    (v1, cross(&v1, &v))
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq;
    #[test]
    fn test_clip() {
        {
            let mut v = vec![
                P3::new(1.0, 2.0, 3.0),
                P3::new(-1.0, -2.0, -3.0),
                P3::new(1.0, 0.0, 3.0),
            ];
            clip(&mut v, Directions::up(), &P3::origin());

            assert_eq!(v[0].x(), ONE);
            assert_eq!(v[0].y(), ZERO);
            assert_eq!(v[0].z(), 3.0);

            assert_eq!(v[1].x(), -ONE);
            assert_eq!(v[1].y(), -2.0);
            assert_eq!(v[1].z(), -3.0);

            assert_eq!(v[2].x(), ONE);
            assert_eq!(v[2].y(), ZERO);
            assert_eq!(v[2].z(), 3.0);
        }

        {
            let mut v = vec![
                P3::new(1.0, 2.0, 3.0),
                P3::new(-1.0, -2.0, -3.0),
                P3::new(1.0, 0.0, 3.0),
            ];
            clip(&mut v, Directions::right(), &P3::origin());

            assert_eq!(v[0].x(), ZERO);
            assert_eq!(v[0].y(), 2.0);
            assert_eq!(v[0].z(), 3.0);

            assert_eq!(v[1].x(), -ONE);
            assert_eq!(v[1].y(), -2.0);
            assert_eq!(v[1].z(), -3.0);

            assert_eq!(v[2].x(), ZERO);
            assert_eq!(v[2].y(), ZERO);
            assert_eq!(v[2].z(), 3.0);
        }

        {
            let mut v = vec![
                P3::new(1.0, 2.0, 3.0),
                P3::new(-1.0, -2.0, -3.0),
                P3::new(1.0, 0.0, 3.0),
            ];
            let normal = Rotation::Z(helper::angle_2_rad(45.0)) * Directions::right();

            clip(&mut v, normal, &P3::origin());

            assert_approx_eq!(dot(&v[0], &normal), ZERO);
            assert_approx_eq!(v[1].x(), -ONE);
            assert_approx_eq!(v[1].y(), -2.0);
            assert_approx_eq!(v[1].z(), -3.0);
            assert_approx_eq!(dot(&v[2], &normal), ZERO);
        }
    }
}
