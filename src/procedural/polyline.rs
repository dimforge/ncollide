use alga::linear::{Rotation, Translation};
use math::{Isometry, Point, Vector};
use na::{self, Real};

/// Geometric description of a polyline.
#[derive(Clone)]
pub struct Polyline<N: Real> {
    /// Coordinates of the polyline vertices.
    coords: Vec<Point<N>>,
    /// Coordinates of the polyline normals.
    normals: Option<Vec<Vector<N>>>,
}

impl<N: Real> Polyline<N> {
    /// Creates a new polyline.
    pub fn new(coords: Vec<Point<N>>, normals: Option<Vec<Vector<N>>>) -> Polyline<N> {
        if let Some(ref ns) = normals {
            assert!(
                coords.len() == ns.len(),
                "There must be exactly one normal per vertex."
            );
        }

        Polyline {
            coords: coords,
            normals: normals,
        }
    }
}

impl<N: Real> Polyline<N> {
    /// Moves the polyline data out of it.
    pub fn unwrap(self) -> (Vec<Point<N>>, Option<Vec<Vector<N>>>) {
        (self.coords, self.normals)
    }

    /// The coordinates of this polyline vertices.
    #[inline]
    pub fn coords(&self) -> &[Point<N>] {
        &self.coords[..]
    }

    /// The mutable coordinates of this polyline vertices.
    #[inline]
    pub fn coords_mut(&mut self) -> &mut [Point<N>] {
        &mut self.coords[..]
    }

    /// The normals of this polyline vertices.
    #[inline]
    pub fn normals(&self) -> Option<&[Vector<N>]> {
        match self.normals {
            Some(ref ns) => Some(&ns[..]),
            None => None,
        }
    }

    /// The mutable normals of this polyline vertices.
    #[inline]
    pub fn normals_mut(&mut self) -> Option<&mut [Vector<N>]> {
        match self.normals {
            Some(ref mut ns) => Some(&mut ns[..]),
            None => None,
        }
    }

    /// Translates each vertex of this polyline.
    pub fn translate_by<T: Translation<Point<N>>>(&mut self, t: &T) {
        for c in self.coords.iter_mut() {
            *c = t.transform_point(c);
        }
    }

    /// Rotates each vertex and normal of this polyline.
    pub fn rotate_by<R: Rotation<Point<N>>>(&mut self, r: &R) {
        for c in self.coords.iter_mut() {
            *c = r.transform_point(c);
        }

        for n in self.normals.iter_mut() {
            for n in n.iter_mut() {
                *n = r.transform_vector(n);
            }
        }
    }

    /// Transforms each vertex and rotates each normal of this polyline.
    pub fn transform_by(&mut self, t: &Isometry<N>) {
        for c in self.coords.iter_mut() {
            *c = t * *c;
        }

        for n in self.normals.iter_mut() {
            for n in n.iter_mut() {
                *n = t * &*n;
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

impl<N: Real> Polyline<N> {
    /// Scales each vertex of this mesh.
    #[inline]
    pub fn scale_by(&mut self, s: &Vector<N>) {
        for c in self.coords.iter_mut() {
            for i in 0..na::dimension::<Vector<N>>() {
                c[i] = (*c)[i] * s[i];
            }
        }
        // FIXME: do something for the normals?
    }
}
