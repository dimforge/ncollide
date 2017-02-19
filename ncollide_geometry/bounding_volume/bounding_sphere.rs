//! Bounding sphere.

use num::Zero;

use alga::linear::NormedSpace;
use na;
use bounding_volume::{BoundingVolume, HasBoundingVolume};
use math::{Point, Isometry};

// Seems useful to help type inference. See issue #84.
/// Computes the bounding sphere of a shape `g` transformed by `m`.
///
/// Same as `g.bounding_sphere(m)`.
pub fn bounding_sphere<P, M, G: ?Sized>(g: &G, m: &M) -> BoundingSphere<P>
    where P: Point,
          G: HasBoundingVolume<M, BoundingSphere<P>> {
    g.bounding_volume(m)
}

/// A Bounding Sphere.
#[derive(Debug, PartialEq, Clone, RustcEncodable, RustcDecodable)]
pub struct BoundingSphere<P: Point> {
    center: P,
    radius: P::Real
}

impl<P> BoundingSphere<P>
    where P: Point {
    /// Creates a new bounding sphere.
    pub fn new(center: P, radius: P::Real) -> BoundingSphere<P> {
        BoundingSphere {
            center: center,
            radius: radius
        }
    }

    /// The bounding sphere center.
    #[inline]
    pub fn center(&self) -> &P {
        &self.center
    }

    /// The bounding sphere radius.
    #[inline]
    pub fn radius(&self) -> P::Real {
        self.radius
    }

    /// Transforms this bounding sphere by `m`.
    #[inline]
    pub fn transform_by<M: Isometry<P>>(&self, m: &M) -> BoundingSphere<P> {
        BoundingSphere::new(m.transform_point(&self.center), self.radius)
    }
}

impl<P: Point> BoundingVolume<P> for BoundingSphere<P> {
    #[inline]
    fn center(&self) -> P {
        *self.center()
    }

    #[inline]
    fn intersects(&self, other: &BoundingSphere<P>) -> bool {
        // FIXME: refactor that with the code from narrow_phase::ball_ball::collide(...) ?
        let delta_pos  = other.center - self.center;
        let distance_squared = na::norm_squared(&delta_pos);
        let sum_radius = self.radius + other.radius;

        distance_squared <= sum_radius * sum_radius
    }

    #[inline]
    fn contains(&self, other: &BoundingSphere<P>) -> bool {
        let delta_pos = other.center - self.center;
        let distance  = na::norm(&delta_pos);

        distance + other.radius <= self.radius
    }

    #[inline]
    fn merge(&mut self, other: &BoundingSphere<P>) {
        let mut dir = *other.center() - *self.center();
        let norm    = dir.normalize_mut();

        if norm.is_zero() {
            if other.radius > self.radius {
                self.radius = other.radius
            }
        }
        else {
            let s_center_dir = na::dot(&self.center.coordinates(), &dir);
            let o_center_dir = na::dot(&other.center.coordinates(), &dir);

            let right;
            let left;

            if s_center_dir + self.radius > o_center_dir + other.radius {
                right = self.center + dir * self.radius;
            }
            else {
                right = other.center + dir * other.radius;
            }

            if -s_center_dir + self.radius > -o_center_dir + other.radius {
                left = self.center.translate_by(&(-dir * self.radius));
            }
            else {
                left = other.center.translate_by(&(-dir * other.radius));
            }

            self.center = na::center(&left, &right);
            self.radius = na::distance(&right, &self.center);
        }
    }

    #[inline]
    fn merged(&self, other: &BoundingSphere<P>) -> BoundingSphere<P> {
        let mut res = self.clone();

        res.merge(other);

        res
    }

    #[inline]
    fn loosen(&mut self, amount: P::Real) {
        assert!(amount >= na::zero(), "The loosening margin must be positive.");
        self.radius = self.radius + amount
    }

    #[inline]
    fn loosened(&self, amount: P::Real) -> BoundingSphere<P> {
        assert!(amount >= na::zero(), "The loosening margin must be positive.");
        BoundingSphere::new(self.center, self.radius + amount)
    }

    #[inline]
    fn tighten(&mut self, amount: P::Real) {
        assert!(amount >= na::zero(), "The tightening margin must be positive.");
        assert!(amount <= self.radius, "The tightening margin is to large.");
        self.radius = self.radius - amount
    }

    #[inline]
    fn tightened(&self, amount: P::Real) -> BoundingSphere<P> {
        assert!(amount >= na::zero(), "The tightening margin must be positive.");
        assert!(amount <= self.radius, "The tightening margin is to large.");
        BoundingSphere::new(self.center, self.radius - amount)
    }
}

//  impl<P> Translation<P::Vector> for BoundingSphere<P>
//      where P: Point {
//      #[inline]
//      fn translation(&self) -> P::Vector {
//          self.center.coordinates()
//      }
//  
//      #[inline]
//      fn inverse_translation(&self) -> P::Vector {
//          -self.translation()
//      }
//  
//      #[inline]
//      fn append_translation_mut(&mut self, dv: &P::Vector) {
//          self.center = self.center + *dv
//      }
//  
//      #[inline]
//      fn append_translation(&self, dv: &P::Vector) -> BoundingSphere<P> {
//          BoundingSphere::new(self.center + *dv, self.radius)
//      }
//  
//      #[inline]
//      fn prepend_translation_mut(&mut self, dv: &P::Vector) {
//          self.append_translation_mut(dv)
//      }
//  
//      #[inline]
//      fn prepend_translation(&self, dv: &P::Vector) -> BoundingSphere<P> {
//          self.append_translation(dv)
//      }
//  
//      #[inline]
//      fn set_translation(&mut self, v: P::Vector) {
//          self.center = P::origin() + v
//      }
//  }
