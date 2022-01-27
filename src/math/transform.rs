use super::{Mat3, Vec3, P3};

/**
 * La class Transform contient une Mat3 pour la rotation et un Vec3 pour la translation.
 * Elle permet de changer un point ou un vecteur de repère, passer du repère local vers monde par exemple.
 * Donc elle doit fournir les opérations nécessaires :
 *
 */

pub struct Transform {
    rotation: Mat3,
    translation: Vec3,
}

impl Transform {
    pub fn new() -> Transform {
        Transform {
            rotation: Mat3::zero(),
            translation: Vec3::zero(),
        }
    }
    pub fn transform(&self, point: &P3) -> P3 {
        &(&self.rotation * point) + &self.translation
    }
    pub fn rotate(&self, point: &P3) -> P3 {
        &self.rotation * point
    }
    pub fn rotate_vec(&self, vec: &Vec3) -> Vec3 {
        &self.rotation * vec
    }
    pub fn translate(&self, point: &P3) -> P3 {
        point + &self.translation
    }
}
