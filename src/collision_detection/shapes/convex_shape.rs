use super::{Shape, ShapeType};
use crate::math::{Point, Real, Vector};

pub trait ConvexShape: Shape {}

// =========================== CUBE =========================== //
pub struct Cube {
    radius: Real,
}

impl Cube {
    pub fn new(side_length: Real) -> Cube {
        Cube {
            radius: side_length / (2 as Real),
        }
    }
    // returns the vertices of the cube centered at the origin (0,0,0)
    fn relative_vertex(&self) -> Vec<Point> {
        let r = self.radius;

        vec![
            // Nearest face along z
            Point::new(r, r, r),
            Point::new(-r, r, r),
            Point::new(-r, -r, r),
            Point::new(r, -r, r),
            // Farest face along z
            Point::new(r, r, -r),
            Point::new(-r, r, -r),
            Point::new(-r, -r, -r),
            Point::new(r, -r, -r),
        ]
    }
}

impl Shape for Cube {
    fn inertia_matrix(&self) -> Vector {
        Vector::zeros()
    }
    fn shape_type(&self) -> ShapeType {
        ShapeType::Cube
    }
}
impl ConvexShape for Cube {}

// =========================== RECTANGLE =========================== //

pub struct Rectangle {
    x_side_length: Real,
    y_side_length: Real,
    z_side_length: Real,
}

impl Shape for Rectangle {
    fn inertia_matrix(&self) -> Vector {
        Vector::zeros()
    }
    fn shape_type(&self) -> ShapeType {
        ShapeType::Rectangle
    }
}
impl ConvexShape for Rectangle {}

// =========================== CONVEX MESH =========================== //

pub struct ConvexMesh {}

impl Shape for ConvexMesh {
    fn inertia_matrix(&self) -> Vector {
        Vector::zeros()
    }
    fn shape_type(&self) -> ShapeType {
        ShapeType::ConvexMesh
    }
}
impl ConvexShape for ConvexMesh {}
