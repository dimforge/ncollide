use crate::math::{Isometry, Point, Rotation, Translation, Vector, DIM};
use na::{self, RealField};

/// Geometric description of a polyline.
#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Polyline<N: RealField> {
    /// Coordinates of the polyline vertices.
    coords: Vec<Point<N>>,
    /// Coordinates of the polyline normals.
    normals: Option<Vec<Vector<N>>>,
}

impl<N: RealField> Polyline<N> {
    /// Creates a new polyline.
    pub fn new(coords: Vec<Point<N>>, normals: Option<Vec<Vector<N>>>) -> Polyline<N> {
        if let Some(ref ns) = normals {
            assert!(
                coords.len() == ns.len(),
                "There must be exactly one normal per vertex."
            );
        }

        Polyline { coords, normals }
    }
}

impl<N: RealField> Polyline<N> {
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
        self.normals.as_ref().map(Vec::as_slice)
    }

    /// The mutable normals of this polyline vertices.
    #[inline]
    pub fn normals_mut(&mut self) -> Option<&mut [Vector<N>]> {
        self.normals.as_mut().map(Vec::as_mut_slice)
    }

    /// Translates each vertex of this polyline.
    pub fn translate_by(&mut self, t: &Translation<N>) {
        for c in self.coords.iter_mut() {
            *c = t * &*c;
        }
    }

    /// Rotates each vertex and normal of this polyline.
    pub fn rotate_by(&mut self, r: &Rotation<N>) {
        for c in self.coords.iter_mut() {
            *c = r * &*c;
        }

        for n in self.normals.iter_mut() {
            for n in n.iter_mut() {
                *n = r * &*n;
            }
        }
    }

    /// Transforms each vertex and rotates each normal of this polyline.
    pub fn transform_by(&mut self, t: &Isometry<N>) {
        for c in self.coords.iter_mut() {
            *c = t * &*c;
        }

        for n in self.normals.iter_mut() {
            for n in n.iter_mut() {
                *n = t * &*n;
            }
        }
    }

    /// Apply a transformation to every vertex and normal of this polyline and returns it.
    #[inline]
    pub fn transformed(mut self, t: &Isometry<N>) -> Self {
        self.transform_by(t);
        self
    }

    /// Scales each vertex of this polyline.
    pub fn scale_by_scalar(&mut self, s: &N) {
        for c in self.coords.iter_mut() {
            *c = *c * *s
        }
        // FIXME: do something for the normals?
    }

    /// Scales each vertex of this mesh.
    #[inline]
    pub fn scale_by(&mut self, s: &Vector<N>) {
        for c in self.coords.iter_mut() {
            for i in 0..DIM {
                c[i] = (*c)[i] * s[i];
            }
        }
        // FIXME: do something for the normals?
    }

    /// Apply a scaling to every vertex and normal of this polyline and returns it.
    #[inline]
    pub fn scaled(mut self, s: &Vector<N>) -> Self {
        self.scale_by(s);
        self
    }
}
