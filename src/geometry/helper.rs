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
