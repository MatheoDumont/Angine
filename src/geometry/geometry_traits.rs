use crate::math::{math_essentials::*, Mat3};

pub struct EdgeIndex {
    // vertex index
    pub vi1: usize,
    pub vi2: usize,
}

pub struct FaceIndex {
    // vertices indices
    pub v_i: Vec<usize>,
}

pub struct PolyhedronSizes {
    pub vertices: usize,
    pub edges: usize,
    pub faces: usize,
}

pub trait PolyhedronTrait {
    // number of vertices, edges, faces
    fn sizes(&self) -> PolyhedronSizes;
    fn local_vertices_ref(&self) -> &Vec<P3>;
    fn local_vertex_ref(&self, vertex_idx: usize) -> &P3;

    fn transformed_vertices(&self) -> Vec<P3> {
        let mut vec = Vec::<P3>::new();
        vec.reserve(self.sizes().vertices);

        for v in self.local_vertices_ref() {
            vec.push(self.transform_ref().transform(&v));
        }

        vec
        // vec![
        //     self.transform.transform(&P3::new(xl, yl, zl)), // far top right
        //     self.transform.transform(&P3::new(-xl, yl, zl)), // far top left
        //     self.transform.transform(&P3::new(-xl, -yl, zl)), // far bot left
        //     self.transform.transform(&P3::new(xl, -yl, zl)), // far bot right
        //     self.transform.transform(&P3::new(xl, yl, -zl)), // near top right
        //     self.transform.transform(&P3::new(-xl, yl, -zl)), // near top left
        //     self.transform.transform(&P3::new(-xl, -yl, -zl)), // near bot left
        //     self.transform.transform(&P3::new(xl, -yl, -zl)), // near bot right
        // ]
    }

    fn transformed_vertex(&self, vertex_idx: usize) -> P3 {
        self.transform_ref()
            .transform(&self.local_vertex_ref(vertex_idx))
    }
    // an edge is as the indices of the two points, from vertices()
    fn edges_ref(&self) -> &Vec<EdgeIndex>;
    // a face is described as a Vec of vertex index, the order is trigonometric
    fn faces_ref(&self) -> &Vec<FaceIndex>;
    // face_index is an index of the Vec<Faces> returned by faces()
    fn face_normal(&self, face_index: usize) -> Vec3;
    fn computed_face_normal(&self, face_index: usize) -> Vec3 {
        let face_indices = &self.faces_ref()[face_index];
        let rotation = self.transform_ref().rotation;

        let p0 = self
            .transform_ref()
            .transform_vec(self.local_vertex_ref(face_indices.v_i[0]));
        let p1 = self
            .transform_ref()
            .transform_vec(self.local_vertex_ref(face_indices.v_i[1]));
        let p2 = self
            .transform_ref()
            .transform_vec(self.local_vertex_ref(face_indices.v_i[2]));
        let d1 = p2 - p1;
        let d2 = p0 - p1;
        let mut normal = cross(&d1, &d2);
        normalize(&mut normal);

        // reoriente la normale pour partir de A
        if dot(&normal, &p0) < 0.0 {
            normal = -normal;
        }
        normal
    }
    fn transform_ref(&self) -> &Transform;

    fn edge_direction(&self, edge_idx: usize) -> Vec3 {
        let edge = &self.edges_ref()[edge_idx];
        // let v = self.local_vertex_ref(edge.vi2) - self.local_vertex_ref(edge.vi1);
        // self.transform_ref().transform_vec(&v)
        let mut d = self.transformed_vertex(edge.vi2) - self.transformed_vertex(edge.vi1);
        normalize(&mut d);
        d
    }
    // the separating axis and his index in the normals
    fn sat_separating_axis(&self) -> Vec<usize>;
}
