use alga::linear::{Rotation, Translation};
use na;
use math::{Isometry, Point};

/// Geometric description of a polyline.
#[derive(Clone)]
pub struct Polyline<P: Point> {
    /// Coordinates of the polyline vertices.
    coords: Vec<P>,
    /// Coordinates of the polyline normals.
    normals: Option<Vec<P::Vector>>,
}

impl<P: Point> Polyline<P> {
    /// Creates a new polyline.
    pub fn new(coords: Vec<P>, normals: Option<Vec<P::Vector>>) -> Polyline<P> {
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

impl<P: Point> Polyline<P> {
    /// Moves the polyline data out of it.
    pub fn unwrap(self) -> (Vec<P>, Option<Vec<P::Vector>>) {
        (self.coords, self.normals)
    }

    /// The coordinates of this polyline vertices.
    #[inline]
    pub fn coords(&self) -> &[P] {
        &self.coords[..]
    }

    /// The mutable coordinates of this polyline vertices.
    #[inline]
    pub fn coords_mut(&mut self) -> &mut [P] {
        &mut self.coords[..]
    }

    /// The normals of this polyline vertices.
    #[inline]
    pub fn normals(&self) -> Option<&[P::Vector]> {
        match self.normals {
            Some(ref ns) => Some(&ns[..]),
            None => None,
        }
    }

    /// The mutable normals of this polyline vertices.
    #[inline]
    pub fn normals_mut(&mut self) -> Option<&mut [P::Vector]> {
        match self.normals {
            Some(ref mut ns) => Some(&mut ns[..]),
            None => None,
        }
    }

    /// Translates each vertex of this polyline.
    pub fn translate_by<T: Translation<P>>(&mut self, t: &T) {
        for c in self.coords.iter_mut() {
            *c = t.transform_point(c);
        }
    }

    /// Rotates each vertex and normal of this polyline.
    pub fn rotate_by<R: Rotation<P>>(&mut self, r: &R) {
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
    pub fn transform_by<T: Isometry<P>>(&mut self, t: &T) {
        for c in self.coords.iter_mut() {
            *c = t.transform_point(c);
        }

        for n in self.normals.iter_mut() {
            for n in n.iter_mut() {
                *n = t.rotate_vector(n);
            }
        }
    }

    /// Scales each vertex of this polyline.
    pub fn scale_by_scalar(&mut self, s: &P::Real) {
        for c in self.coords.iter_mut() {
            *c = *c * *s
        }
        // FIXME: do something for the normals?
    }
}

impl<P> Polyline<P>
where
    P: Point,
{
    /// Scales each vertex of this mesh.
    #[inline]
    pub fn scale_by(&mut self, s: &P::Vector) {
        for c in self.coords.iter_mut() {
            for i in 0..na::dimension::<P::Vector>() {
                c[i] = (*c)[i] * s[i];
            }
        }
        // FIXME: do something for the normals?
    }
}
