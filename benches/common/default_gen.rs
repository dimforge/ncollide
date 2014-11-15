use std::rand::{Rand, Rng};
use na;
use ncollide::bounding_volume::{AABB, BoundingSphere};
use ncollide::shape::{Ball, Cuboid, Capsule, Cone, Cylinder, Segment, Triangle, Convex};
use ncollide::math::{Scalar, Point, Vect};
use ncollide::ray::Ray;

trait DefaultGen {
    fn generate<R: Rng>(rng: &mut R) -> Self;
}

pub fn generate<T: DefaultGen, R: Rng>(rng: &mut R) -> T {
    DefaultGen::generate(rng)
}

impl<T: Rand> DefaultGen for T {
    fn generate<R: Rng>(rng: &mut R) -> T {
        rng.gen::<T>()
    }
}

impl<N: Scalar> DefaultGen for Ball<N> {
    fn generate<R: Rng>(rng: &mut R) -> Ball<N> {
        Ball::new(na::abs(&rng.gen::<N>()))
    }
}

impl<N: Scalar, V: Vect<N>> DefaultGen for Cuboid<V> {
    fn generate<R: Rng>(rng: &mut R) -> Cuboid<V> {
        Cuboid::new(na::abs(&rng.gen::<V>()))
    }
}

impl<N: Scalar> DefaultGen for Capsule<N> {
    fn generate<R: Rng>(rng: &mut R) -> Capsule<N> {
        Capsule::new(na::abs(&rng.gen::<N>()), na::abs(&rng.gen::<N>()))
    }
}

impl<N: Scalar> DefaultGen for Cone<N> {
    fn generate<R: Rng>(rng: &mut R) -> Cone<N> {
        Cone::new(na::abs(&rng.gen::<N>()), na::abs(&rng.gen::<N>()))
    }
}

impl<N: Scalar> DefaultGen for Cylinder<N> {
    fn generate<R: Rng>(rng: &mut R) -> Cylinder<N> {
        Cylinder::new(na::abs(&rng.gen::<N>()), na::abs(&rng.gen::<N>()))
    }
}

impl<N, P, V> DefaultGen for Segment<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn generate<R: Rng>(rng: &mut R) -> Segment<P> {
        Segment::new(na::orig::<P>() + rng.gen(), na::orig::<P>() + rng.gen())
    }
}

impl<N, P, V> DefaultGen for Triangle<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn generate<R: Rng>(rng: &mut R) -> Triangle<P> {
        Triangle::new(na::orig::<P>() + rng.gen(), na::orig::<P>() + rng.gen(), na::orig::<P>() + rng.gen())
    }
}

impl<N, P, V> DefaultGen for Convex<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn generate<R: Rng>(rng: &mut R) -> Convex<P> {
        // It is recommanded to have at most 100 points.
        // Otherwise, a smarter structure like the DK hierarchy would be needed.
        let pts = Vec::from_fn(100, |_| na::orig::<P>() + rng.gen::<V>());
        Convex::new(pts)
    }
}

impl<N, P, V> DefaultGen for Ray<P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn generate<R: Rng>(rng: &mut R) -> Ray<P, V> {
        // The generate ray will always point to the origin.
        let shift = rng.gen::<V>() * na::cast(10.0f64);
        Ray::new(na::orig::<P>() + shift, -shift)
    }
}

impl<N, P, V> DefaultGen for AABB<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn generate<R: Rng>(rng: &mut R) -> AABB<P> {
        // an AABB centered at the origin.
        let half_extents = na::abs(&rng.gen::<V>());
        AABB::new(na::orig::<P>() + (-half_extents), na::orig::<P>() + half_extents)
    }
}

impl<N, P, V> DefaultGen for BoundingSphere<N, P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn generate<R: Rng>(rng: &mut R) -> BoundingSphere<N, P> {
        // a bounding sphere centered at the origin.
        BoundingSphere::new(na::orig(), na::abs(&rng.gen::<N>()))
    }
}
