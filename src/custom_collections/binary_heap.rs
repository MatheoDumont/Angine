use std::cmp::{PartialEq, PartialOrd};

// source = https://perso.liris.cnrs.fr/vincent.nivoliers/lifap6/Supports/TD/td06.pdf

pub struct BinaryHeap<K, V> {
    data: Vec<Option<Cell<K, V>>>,
    length: usize,
}
#[derive(Clone)]
pub struct Cell<K, V> {
    key: K,
    value: V,
}

impl<K, V> Cell<K, V> {
    fn tuplify(self) -> (K, V) {
        (self.key, self.value)
    }
}

impl<K, V> Cell<K, V> {
    pub fn new(key: K, value: V) -> Cell<K, V> {
        Cell { key, value }
    }
}

fn parent_index(i: usize) -> usize {
    (i - 1) / 2
}

fn left_child_index(i: usize) -> usize {
    i * 2 + 1
}

fn right_child_index(i: usize) -> usize {
    i * 2 + 2
}

impl<K, V> BinaryHeap<K, V>
where
    K: PartialOrd + PartialEq + Clone,
    V: Clone,
{
    pub fn new(capacity: usize) -> BinaryHeap<K, V> {
        let mut data = Vec::<Option<Cell<K, V>>>::with_capacity(capacity);
        data.resize(capacity, None);
        BinaryHeap { data, length: 0 }
    }

    pub fn at(&self, i: usize) -> Option<&Cell<K, V>> {
        self.data[i].as_ref()
    }

    pub fn append(&mut self, key: K, value: V) -> usize {
        let mut i = self.length;
        self.length += 1;
        while i > 0 && key > self.parent(i).key {
            let p_i = parent_index(i);
            self.data[i] = self.data[p_i].clone();
            i = p_i;
        }
        self.data[i] = Some(Cell::new(key, value));
        i
    }

    /**
     * pop the cell at the top of the heap
     * crash if self.length == 0
     */
    pub fn pop(&mut self) -> (K, V) {
        if self.length == 1 {
            self.length = 0;
            let e = self.data[0].clone().unwrap();
            self.data[0] = None;
            return e.tuplify();
        }
        let popped = self.data[0].clone();

        let mut end = self.length - 1;
        let e = self.data[end].clone().unwrap();
        self.data[end] = None;
        self.length -= 1;
        end -= 1;

        let mut i = 0;

        // check out of memory and superior child node
        while (left_child_index(i) < self.length && e.key < self.left_child(i).key)
            || (right_child_index(i) < self.length && e.key < self.right_child(i).key)
        {
            if left_child_index(i) < self.length
                && e.key < self.left_child(i).key
                && self.left_child(i).key >= self.right_child(i).key
            {
                let lci = left_child_index(i);
                self.data[i] = self.data[lci].clone();
                i = lci;
            } else {
                let rci = right_child_index(i);
                self.data[i] = self.data[rci].clone();
                i = rci;
            }
        }

        self.data[i] = Some(e);

        popped.unwrap().tuplify()
    }

    fn parent(&self, i: usize) -> &Cell<K, V> {
        &self.data[parent_index(i)].as_ref().unwrap()
    }

    fn left_child(&self, i: usize) -> &Cell<K, V> {
        &self.data[left_child_index(i)].as_ref().unwrap()
    }

    fn right_child(&self, i: usize) -> &Cell<K, V> {
        &self.data[right_child_index(i)].as_ref().unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_append() {
        let mut bh = BinaryHeap::<i32, i32>::new(5);

        let mut i = bh.append(1, 1);
        assert_eq!(i, 0);
        assert_eq!(bh.at(0).unwrap().key, 1);
        assert_eq!(bh.at(0).unwrap().value, 1);

        i = bh.append(2, 2);
        assert_eq!(i, 0);
        assert_eq!(bh.at(0).unwrap().key, 2);
        assert_eq!(bh.at(0).unwrap().value, 2);
        assert_eq!(bh.at(1).unwrap().key, 1);
        assert_eq!(bh.at(1).unwrap().value, 1);

        i = bh.append(1, 2);
        assert_eq!(i, 2);
        assert_eq!(bh.at(2).unwrap().key, 1);
        assert_eq!(bh.at(2).unwrap().value, 2);
    }

    #[test]
    fn test_pop() {
        let mut bh = BinaryHeap::<f32, i32>::new(5);
        bh.append(1.0, 1);
        bh.append(2.0, 2);
        bh.append(1.5, 2);
        bh.append(3.0, 2);
        bh.append(1.0, 2);

        assert_eq!(bh.pop().0, 3.0);
        assert_eq!(bh.pop().0, 2.0);
        assert_eq!(bh.pop().0, 1.5);
        assert_eq!(bh.pop().0, 1.0);
        assert_eq!(bh.pop().0, 1.0);
    }
}
