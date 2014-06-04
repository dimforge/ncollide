use nalgebra::na::{Translate, Rotate, Transform, FloatVec};

/// Geometric description of a polyline.
#[deriving(Clone)]
pub struct Polyline<N, V> {
    /// Coordinates of the polyline vertices.
    pub coords:  Vec<V>,
    /// Coordinates of the polyline normals.
    pub normals: Option<Vec<V>>
}

impl<N, V: FloatVec<N>> Polyline<N, V> {
    /// Creates a new polyline.
    pub fn new(coords: Vec<V>, normals: Option<Vec<V>>) -> Polyline<N, V> {
        Polyline {
            coords:  coords,
            normals: normals
        }
    }

    /// Translates each vertex of this polyline.
    pub fn translate_by<T: Translate<V>>(&mut self, t: &T) {
        for c in self.coords.mut_iter() {
            *c = t.translate(c);
        }
    }

    /// Rotates each vertex and normal of this polyline.
    pub fn rotate_by<R: Rotate<V>>(&mut self, r: &R) {
        for c in self.coords.mut_iter() {
            *c = r.rotate(c);
        }

        for n in self.normals.mut_iter() {
            for n in n.mut_iter() {
                *n = r.rotate(n);
            }
        }
    }

    /// Transforms each vertex and rotates each normal of this polyline.
    pub fn transform_by<T: Transform<V> + Rotate<V>>(&mut self, t: &T) {
        for c in self.coords.mut_iter() {
            *c = t.transform(c);
        }

        for n in self.normals.mut_iter() {
            for n in n.mut_iter() {
                *n = t.rotate(n);
            }
        }
    }

    /// Scales each vertex of this polyline.
    pub fn scale_by_scalar(&mut self, s: &N) {
        for c in self.coords.mut_iter() {
            *c = *c * *s
        }
        // FIXME: do something for the normals?
    }
}
