use crate::math::{Real, Vec3, P3, TWO};

pub fn closest_to_p(p: &P3, pts: &Vec<P3>) -> usize {
    let mut closest_vertex = 0;
    let mut d = (&pts[closest_vertex] - p).norm_squared();
    for i in 1..pts.len() {
        let dd = (p - &pts[i]).norm_squared();
        if d > dd {
            d = dd;
            closest_vertex = i;
        }
    }
    closest_vertex
}

pub fn closest_projection(normal: &Vec3, points: &Vec<P3>) -> (Real, usize) {
    let mut smallest_distance = f32::MAX;
    let mut closest_point_index = 0;
    for i in 0..points.len() {
        let x = normal.dot_point(&points[i]);
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
        midpoint[0] += p[0];
        midpoint[1] += p[1];
        midpoint[2] += p[2];
    }
    let size = points.len() as Real;
    midpoint[0] = midpoint[0] / size;
    midpoint[1] = midpoint[1] / size;
    midpoint[2] = midpoint[2] / size;
    midpoint
}
