use std::rand::{Rand, Rng};
use na;
use ncollide::bounding_volume::{AABB, BoundingSphere};
use ncollide::shape::{Ball, Cuboid, Capsule, Cone, Cylinder};
use ncollide::math::{Scalar, Point, Vect};
use ncollide::ray::Ray;

trait MyRand {
    fn random<R: Rng>(rng: &mut R) -> Self;
}

pub fn random<T: MyRand, R: Rng>(rng: &mut R) -> T {
    MyRand::random(rng)
}

impl<T: Rand> MyRand for T {
    fn random<R: Rng>(rng: &mut R) -> T {
        rng.gen::<T>()
    }
}

impl<N: Scalar> MyRand for Ball<N> {
    fn random<R: Rng>(rng: &mut R) -> Ball<N> {
        Ball::new(rng.gen::<N>().abs())
    }
}

impl<N: Scalar, V: Vect<N>> MyRand for Cuboid<V> {
    fn random<R: Rng>(rng: &mut R) -> Cuboid<V> {
        Cuboid::new(na::abs(&rng.gen::<V>()))
    }
}

impl<N: Scalar> MyRand for Capsule<N> {
    fn random<R: Rng>(rng: &mut R) -> Capsule<N> {
        Capsule::new(rng.gen::<N>().abs(), rng.gen::<N>().abs())
    }
}

impl<N: Scalar> MyRand for Cone<N> {
    fn random<R: Rng>(rng: &mut R) -> Cone<N> {
        Cone::new(rng.gen::<N>().abs(), rng.gen::<N>().abs())
    }
}

impl<N: Scalar> MyRand for Cylinder<N> {
    fn random<R: Rng>(rng: &mut R) -> Cylinder<N> {
        Cylinder::new(rng.gen::<N>().abs(), rng.gen::<N>().abs())
    }
}

impl<N, P, V> MyRand for Ray<P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn random<R: Rng>(rng: &mut R) -> Ray<P, V> {
        // The random ray will always point to the origin.
        let shift = rng.gen::<V>() * na::cast(10.0f64);
        Ray::new(na::orig::<P>() + shift, -shift)
    }
}

impl<N, P, V> MyRand for AABB<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn random<R: Rng>(rng: &mut R) -> AABB<P> {
        // an AABB centered at the origin.
        let half_extents = na::abs(&rng.gen::<V>());
        AABB::new(na::orig::<P>() + (-half_extents), na::orig::<P>() + half_extents)
    }
}

impl<N, P, V> MyRand for BoundingSphere<N, P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn random<R: Rng>(rng: &mut R) -> BoundingSphere<N, P> {
        // a bounding sphere centered at the origin.
        BoundingSphere::new(na::orig(), rng.gen::<N>().abs())
    }
}
