use super::{Vec3, P3};

pub struct EdgeIndex {
    // vertex index
    pub vi1: usize,
    pub vi2: usize,
}

pub struct FaceIndex {
    // vertices indices
    pub v_i: Vec<usize>,
}

pub trait Polyhedron {
    // number of vertices, edges, faces
    fn sizes(&self) -> (usize, usize, usize);
    fn vertices(&self) -> Vec<P3>;
    // a face is described as a Vec of vertex index, the order is trigonometric
    fn faces(&self) -> Vec<FaceIndex>;
    // an edge is as the indices of the two points, from vertices()
    fn edges(&self) -> Vec<EdgeIndex>;
    // face_index is an index of the Vec<Faces> returned by faces()
    fn face_normal(&self, face_index: usize) -> Vec3;
}
