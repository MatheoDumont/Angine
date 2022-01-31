use super::{Real, Vec3, P3};
use std::ops::Mul;

/**
 * Implémentation unique de Matrice carré 3x3.
 *
 * Que doivent fournir les matrices ?
 *      Contenir des informations comme des rotations, pouvoir être multipliées avec des vecteurs.
 *      Fournir quelques opérations typiques : créer simplement, par copie, transposer, accéder simplement aux élements et les modifier.
 *      Point de départ pour créer l'objet Transform composé d'une Mat3 pour la rotation et d'un Vecteur pour la translation
 *      Mat3 pourra etre améliorer pour fournir des méthodes qui permettent d'orienter simplement, sous la forme
 *          d'un autre type qui a un trait Rotation par exemple, voir meme des quaternions (qui sait)
 *      La type de données est uniquement Real, définit dans math.rs, qui est commun au projet entier.
 *
 */

pub struct Mat3 {
    data: [[Real; 3]; 3],
}

impl Mat3 {
    pub fn identity() -> Mat3 {
        let zero = 0 as Real;
        let one = 1 as Real;

        Mat3 {
            data: [[one, zero, zero], [zero, one, zero], [zero, zero, one]],
        }
    }
    pub fn zero() -> Mat3 {
        Mat3 {
            data: [[0 as Real; 3]; 3],
        }
    }

    pub fn value(v: Real) -> Mat3 {
        Mat3 { data: [[v; 3]; 3] }
    }

    pub fn diag(v: Vec3) -> Mat3 {
        let zero = 0 as Real;

        Mat3 {
            data: [[v.x, zero, zero], [zero, v.y, zero], [zero, zero, v.z]],
        }
    }

    pub fn from_array(arr: [[Real; 3]; 3]) -> Mat3 {
        Mat3 { data: arr }
    }

    pub fn from_mat3(o: &Mat3) -> Mat3 {
        Mat3 {
            data: o.data.clone(),
        }
    }

    pub fn size(&self) -> (usize, usize) {
        (3, 3)
    }

    pub fn col(&self, col: usize) -> Vec3 {
        Vec3 {
            x: self.data[0][col],
            y: self.data[1][col],
            z: self.data[2][col],
        }
    }

    pub fn row(&self, row: usize) -> Vec3 {
        Vec3 {
            x: self.data[row][0],
            y: self.data[row][1],
            z: self.data[row][2],
        }
    }

    pub fn at(&self, row: usize, col: usize) -> Real {
        self.data[row][col]
    }

    pub fn at_ref(&self, row: usize, col: usize) -> &Real {
        &self.data[row][col]
    }

    pub fn at_mut(&mut self, row: usize, col: usize) -> &mut Real {
        &mut self.data[row][col]
    }

    pub fn transpose(&mut self) {
        // on transpose une matrice carré donc on ne touche jamais aux coefs sur la diag
        // on opère juste des 'swap' symétriquement le long de la diag
        let mut tmp = self.data[0][1];
        self.data[0][1] = self.data[1][0];
        self.data[1][0] = tmp;

        tmp = self.data[0][2];
        self.data[0][2] = self.data[2][0];
        self.data[2][0] = tmp;

        tmp = self.data[2][1];
        self.data[2][1] = self.data[1][2];
        self.data[1][2] = tmp;
    }

    pub fn transposed(&self) -> Mat3 {
        // on transpose une matrice carré donc on ne touche jamais aux coefs sur la diag
        // on opère juste des 'swap' symétriquement le long de la diag
        let mut m = Mat3::identity();

        for r in 0..3 {
            for c in 0..3 {
                m.data[r][c] = self.data[c][r];
            }
        }
        m
    }

    /**
     * Return a 2D Matrix filled with the elements of the Mat3 after removing Mat3.row(row) and Mat3.col(col)
     *
     * For example, if row and col are (0,0)
     * | 1 2 3 |     | - - - |
     * | 4 5 6 |  -> | - 5 6 | -> | 5 6 |
     * | 7 8 9 |     | - 8 9 |    | 8 9 |
     *
     * Used to compute the minor of the element at (row, col)
     */
    fn cut(&self, row: usize, col: usize) -> [[Real; 2]; 2] {
        let mut arr = [[0 as Real; 2]; 2];
        let mut i: usize = 0;

        for r in 0..3 {
            if r == row {
                continue;
            }
            for c in 0..3 {
                if c == col {
                    continue;
                }
                arr[i / 2][i % 2] = self.data[r][c];
                i += 1;
            }
        }
        arr
    }

    fn determinant_from_cut(arr: [[Real; 2]; 2]) -> Real {
        arr[0][0] * arr[1][1] - arr[0][1] * arr[1][0]
    }

    fn determinant(&self) -> Real {
        Mat3::determinant_from_cut(self.cut(0, 0)) * self.data[0][0]
            - Mat3::determinant_from_cut(self.cut(0, 1)) * self.data[0][1]
            + Mat3::determinant_from_cut(self.cut(0, 2)) * self.data[0][2]
    }

    pub fn minors(&self) -> Mat3 {
        let mut m = Mat3::identity();

        for r in 0..3 {
            for c in 0..3 {
                let arr = self.cut(r, c);
                m.data[r][c] = Mat3::determinant_from_cut(arr);
            }
        }

        m
    }

    pub fn cofactor_from(minors: &Mat3) -> Mat3 {
        let mut cofactor = Mat3::identity();
        let one_neg = -1 as Real;
        for r in 0..3 {
            for c in 0..3 {
                cofactor.data[r][c] = minors.at(r, c) * one_neg.powi((r + c) as i32) as Real;
            }
        }
        cofactor
    }

    pub fn adjugate_from(cofactors: &Mat3) -> Mat3 {
        cofactors.transposed()
    }

    pub fn inverse(&self) -> Mat3 {
        let det = self.determinant();
        assert!(det != (0 as Real), "Matrice non-inversible, det(mat) = 0");

        let minors = self.minors();
        let cofactors = Mat3::cofactor_from(&minors);
        let adj = Mat3::adjugate_from(&cofactors);
        &adj * (1 as Real / det)
    }
}

#[rustfmt::skip]
impl Mul for &Mat3 {
    type Output = Mat3;
    fn mul(self, r: Self) -> Self::Output {
        let mut m = Mat3::zero();

        *m.at_mut(0, 0) = self.at_ref(0, 0) * r.at_ref(0, 0) // left row0 * right col0
                        + self.at_ref(0, 1) * r.at_ref(1, 0)
                        + self.at_ref(0, 2) * r.at_ref(2, 0);
        *m.at_mut(1, 0) = self.at_ref(1, 0) * r.at_ref(0, 0) // left row1 * right col0
                        + self.at_ref(1, 1) * r.at_ref(1, 0)
                        + self.at_ref(1, 2) * r.at_ref(2, 0);
        *m.at_mut(2, 0) = self.at_ref(2, 0) * r.at_ref(0, 0) // left row2 * right col0
                        + self.at_ref(2, 1) * r.at_ref(1, 0)
                        + self.at_ref(2, 2) * r.at_ref(2, 0);

        *m.at_mut(0, 1) = self.at_ref(0, 0) * r.at_ref(0, 1) // left row0 * right col1
                        + self.at_ref(0, 1) * r.at_ref(1, 1)
                        + self.at_ref(0, 2) * r.at_ref(2, 1);
        *m.at_mut(1, 1) = self.at_ref(1, 0) * r.at_ref(0, 1) // left row1 * right col1
                        + self.at_ref(1, 1) * r.at_ref(1, 1)
                        + self.at_ref(1, 2) * r.at_ref(2, 1);
        *m.at_mut(2, 1) = self.at_ref(2, 0) * r.at_ref(0, 1) // left row2 * right col1
                        + self.at_ref(2, 1) * r.at_ref(1, 1)
                        + self.at_ref(2, 2) * r.at_ref(2, 1);

        *m.at_mut(0, 2) = self.at_ref(0, 0) * r.at_ref(0, 2) // left row0 * right col2
                        + self.at_ref(0, 1) * r.at_ref(1, 2)
                        + self.at_ref(0, 2) * r.at_ref(2, 2);
        *m.at_mut(1, 2) = self.at_ref(1, 0) * r.at_ref(0, 2) // left row1 * right col2
                        + self.at_ref(1, 1) * r.at_ref(1, 2)
                        + self.at_ref(1, 2) * r.at_ref(2, 2);
        *m.at_mut(2, 2) = self.at_ref(2, 0) * r.at_ref(0, 2) // left row2 * right col2
                        + self.at_ref(2, 1) * r.at_ref(1, 2)
                        + self.at_ref(2, 2) * r.at_ref(2, 2);

        m
    }
}

#[rustfmt::skip]
impl Mul for Mat3 {
    type Output = Mat3;
    fn mul(self, r: Self) -> Self::Output {
        let mut m = Mat3::zero();

        *m.at_mut(0, 0) = self.at_ref(0, 0) * r.at_ref(0, 0) // left row0 * right col0
                        + self.at_ref(0, 1) * r.at_ref(1, 0)
                        + self.at_ref(0, 2) * r.at_ref(2, 0);
        *m.at_mut(1, 0) = self.at_ref(1, 0) * r.at_ref(0, 0) // left row1 * right col0
                        + self.at_ref(1, 1) * r.at_ref(1, 0)
                        + self.at_ref(1, 2) * r.at_ref(2, 0);
        *m.at_mut(2, 0) = self.at_ref(2, 0) * r.at_ref(0, 0) // left row2 * right col0
                        + self.at_ref(2, 1) * r.at_ref(1, 0)
                        + self.at_ref(2, 2) * r.at_ref(2, 0);

        *m.at_mut(0, 1) = self.at_ref(0, 0) * r.at_ref(0, 1) // left row0 * right col1
                        + self.at_ref(0, 1) * r.at_ref(1, 1)
                        + self.at_ref(0, 2) * r.at_ref(2, 1);
        *m.at_mut(1, 1) = self.at_ref(1, 0) * r.at_ref(0, 1) // left row1 * right col1
                        + self.at_ref(1, 1) * r.at_ref(1, 1)
                        + self.at_ref(1, 2) * r.at_ref(2, 1);
        *m.at_mut(2, 1) = self.at_ref(2, 0) * r.at_ref(0, 1) // left row2 * right col1
                        + self.at_ref(2, 1) * r.at_ref(1, 1)
                        + self.at_ref(2, 2) * r.at_ref(2, 1);

        *m.at_mut(0, 2) = self.at_ref(0, 0) * r.at_ref(0, 2) // left row0 * right col2
                        + self.at_ref(0, 1) * r.at_ref(1, 2)
                        + self.at_ref(0, 2) * r.at_ref(2, 2);
        *m.at_mut(1, 2) = self.at_ref(1, 0) * r.at_ref(0, 2) // left row1 * right col2
                        + self.at_ref(1, 1) * r.at_ref(1, 2)
                        + self.at_ref(1, 2) * r.at_ref(2, 2);
        *m.at_mut(2, 2) = self.at_ref(2, 0) * r.at_ref(0, 2) // left row2 * right col2
                        + self.at_ref(2, 1) * r.at_ref(1, 2)
                        + self.at_ref(2, 2) * r.at_ref(2, 2);

        m
    }
}

impl Mul<&Vec3> for &Mat3 {
    type Output = Vec3;
    fn mul(self, v: &Vec3) -> Self::Output {
        Vec3 {
            x: self.data[0][0] * v.x + self.data[0][1] * v.y + self.data[0][2] * v.z,
            y: self.data[1][0] * v.x + self.data[1][1] * v.y + self.data[1][2] * v.z,
            z: self.data[2][0] * v.x + self.data[2][1] * v.y + self.data[2][2] * v.z,
        }
    }
}

impl Mul<&P3> for &Mat3 {
    type Output = P3;
    fn mul(self, v: &P3) -> Self::Output {
        P3 {
            x: self.data[0][0] * v.x + self.data[0][1] * v.y + self.data[0][2] * v.z,
            y: self.data[1][0] * v.x + self.data[1][1] * v.y + self.data[1][2] * v.z,
            z: self.data[2][0] * v.x + self.data[2][1] * v.y + self.data[2][2] * v.z,
        }
    }
}

impl Mul<Real> for &Mat3 {
    type Output = Mat3;
    fn mul(self, v: Real) -> Self::Output {
        Mat3::from_array([
            [
                self.data[0][0] * v,
                self.data[0][1] * v,
                self.data[0][2] * v,
            ],
            [
                self.data[1][0] * v,
                self.data[1][1] * v,
                self.data[1][2] * v,
            ],
            [
                self.data[2][0] * v,
                self.data[2][1] * v,
                self.data[2][2] * v,
            ],
        ])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matrix_mul() {
        // cast le tableau en Real
        let arr = [[1, 2, 3], [4, 5, 6], [7, 8, 9]];
        let left = Mat3::from_array(arr.map(|e| e.map(|b| b as Real)));
        let right = Mat3::from_array(arr.map(|e| e.map(|b| b as Real)));

        let l_r0 = left.row(0);
        let l_r1 = left.row(1);
        let l_r2 = left.row(2);

        let r_c0 = right.col(0);
        let r_c1 = right.col(1);
        let r_c2 = right.col(2);

        let m = &left * &right;
        assert_eq!(m.at(0, 0), &l_r0 * &r_c0);
        assert_eq!(m.at(1, 0), &l_r1 * &r_c0);
        assert_eq!(m.at(2, 0), &l_r2 * &r_c0);

        assert_eq!(m.at(0, 1), &l_r0 * &r_c1);
        assert_eq!(m.at(1, 1), &l_r1 * &r_c1);
        assert_eq!(m.at(2, 1), &l_r2 * &r_c1);

        assert_eq!(m.at(0, 2), &l_r0 * &r_c2);
        assert_eq!(m.at(1, 2), &l_r1 * &r_c2);
        assert_eq!(m.at(2, 2), &l_r2 * &r_c2);
    }
    #[test]
    fn cut() {
        let arr = [[1, 2, 3], [4, 5, 6], [7, 8, 9]].map(|e| e.map(|b| b as Real));
        let m = Mat3::from_array(arr);
        let arr = m.cut(0, 0);

        assert_eq!(arr[0][0], 5 as Real);
        assert_eq!(arr[0][1], 6 as Real);
        assert_eq!(arr[1][0], 8 as Real);
        assert_eq!(arr[1][1], 9 as Real);
    }

    #[test]
    fn determinant() {
        {
            let arr = [[1, 2, 3], [4, 5, 6], [7, 8, 9]].map(|e| e.map(|b| b as Real));
            let m = Mat3::from_array(arr);

            assert_eq!(m.determinant(), 0 as Real);
        }

        {
            let arr = [[1, 2, 3], [4, 5, 6], [7, 8, 10]].map(|e| e.map(|b| b as Real));
            let m = Mat3::from_array(arr);

            assert_eq!(m.determinant(), -3 as Real);
        }
    }
    #[test]
    fn inverse() {
        let arr = [[1, 2, 3], [4, 5, 6], [7, 8, 10]].map(|e| e.map(|b| b as Real));
        let m = Mat3::from_array(arr);

        let inv = m.inverse();

        let id = &m * &inv;
        // assert that M*M⁻1 = Identity
        for r in 0..3 {
            for c in 0..3 {
                if r == c {
                    assert_eq!(id.data[r][c], 1 as Real);
                } else {
                    assert_eq!(id.data[r][c], 0 as Real);
                }
            }
        }
    }
}
