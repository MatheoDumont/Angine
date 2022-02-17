use super::P3;

pub trait Mesh {
    fn vertices(&self) -> Vec<P3>;
}
