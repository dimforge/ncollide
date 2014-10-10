use na;
use na::{Translate, Rotate, Transform, FloatVec, FloatPnt, Dim, Indexable};

/// Geometric description of a polyline.
#[deriving(Clone)]
pub struct Polyline<N, P, V> {
    /// Coordinates of the polyline vertices.
    pub coords:  Vec<P>,
    /// Coordinates of the polyline normals.
    pub normals: Option<Vec<V>>
}

impl<N, P, V> Polyline<N, P, V> {
    /// Creates a new polyline.
    pub fn new(coords: Vec<P>, normals: Option<Vec<V>>) -> Polyline<N, P, V> {
        Polyline {
            coords:  coords,
            normals: normals
        }
    }
}

impl<N, P: FloatPnt<N, V>, V: FloatVec<N>> Polyline<N, P, V> {
    /// Translates each vertex of this polyline.
    pub fn translate_by<T: Translate<P>>(&mut self, t: &T) {
        for c in self.coords.iter_mut() {
            *c = t.translate(c);
        }
    }

    /// Rotates each vertex and normal of this polyline.
    // XXX: we should use Rotate<P> instead of the .set_coord.
    // Wa cannot make it a Rotate because the `Rotate` bound cannot appear twice… we have, again,
    // to wait for the trait reform.
    pub fn rotate_by<R: Rotate<V>>(&mut self, r: &R) {
        for c in self.coords.iter_mut() {
            let rc = r.rotate(c.as_vec());
            c.set_coords(rc);
        }

        for n in self.normals.iter_mut() {
            for n in n.iter_mut() {
                *n = r.rotate(n);
            }
        }
    }

    /// Transforms each vertex and rotates each normal of this polyline.
    pub fn transform_by<T: Transform<P> + Rotate<V>>(&mut self, t: &T) {
        for c in self.coords.iter_mut() {
            *c = t.transform(c);
        }

        for n in self.normals.iter_mut() {
            for n in n.iter_mut() {
                *n = t.rotate(n);
            }
        }
    }

    /// Scales each vertex of this polyline.
    pub fn scale_by_scalar(&mut self, s: &N) {
        for c in self.coords.iter_mut() {
            *c = *c * *s
        }
        // FIXME: do something for the normals?
    }
}

impl<N: Mul<N, N>, P: Indexable<uint, N>, V: Dim + Indexable<uint, N>> Polyline<N, P, V> {
    /// Scales each vertex of this mesh.
    #[inline]
    pub fn scale_by(&mut self, s: &V) {
        for c in self.coords.iter_mut() {
            for i in range(0, na::dim::<V>()) {
                let val = c.at(i);
                let mul = s.at(i);
                c.set(i, val * mul);
            }
        }
        // FIXME: do something for the normals?
    }
}
