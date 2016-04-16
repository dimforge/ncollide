use rand::Rng;
use na;
use na::{Vec2, Vec3, Vec4, Pnt2, Pnt3, Pnt4, Mat2, Mat3, Mat4, Iso2, Iso3, Identity};
use ncollide::bounding_volume::{AABB, BoundingSphere};
use ncollide::shape::{Ball, Cuboid, Capsule, Cone, Cylinder, Segment, Triangle, ConvexHull};
use ncollide::math::{Scalar, Point, Vect};
use ncollide::ray::Ray;

pub trait DefaultGen {
    fn generate<R: Rng>(rng: &mut R) -> Self;
}

pub fn generate<T: DefaultGen, R: Rng>(rng: &mut R) -> T {
    DefaultGen::generate(rng)
}

macro_rules! impl_rand_default_gen(
    ($t: ty) => {
        impl DefaultGen for $t {
            fn generate<R: Rng>(rng: &mut R) -> $t {
                rng.gen::<$t>()
            }
        }
    }
);

impl_rand_default_gen!(Vec2<f32>);
impl_rand_default_gen!(Vec3<f32>);
impl_rand_default_gen!(Vec4<f32>);
impl_rand_default_gen!(Pnt2<f32>);
impl_rand_default_gen!(Pnt3<f32>);
impl_rand_default_gen!(Pnt4<f32>);
impl_rand_default_gen!(Mat2<f32>);
impl_rand_default_gen!(Mat3<f32>);
impl_rand_default_gen!(Mat4<f32>);
impl_rand_default_gen!(Iso2<f32>);
impl_rand_default_gen!(Iso3<f32>);
impl_rand_default_gen!(Vec2<f64>);
impl_rand_default_gen!(Vec3<f64>);
impl_rand_default_gen!(Vec4<f64>);
impl_rand_default_gen!(Pnt2<f64>);
impl_rand_default_gen!(Pnt3<f64>);
impl_rand_default_gen!(Pnt4<f64>);
impl_rand_default_gen!(Mat2<f64>);
impl_rand_default_gen!(Mat3<f64>);
impl_rand_default_gen!(Mat4<f64>);
impl_rand_default_gen!(Iso2<f64>);
impl_rand_default_gen!(Iso3<f64>);
impl_rand_default_gen!(f32);
impl_rand_default_gen!(f64);
impl_rand_default_gen!(bool);

impl DefaultGen for Identity {
    fn generate<R: Rng>(_: &mut R) -> Identity {
        Identity::new()
    }
}

impl<N: Scalar> DefaultGen for Ball<N> {
    fn generate<R: Rng>(rng: &mut R) -> Ball<N> {
        Ball::new(na::abs(&rng.gen::<N>()))
    }
}

impl<V: Vect> DefaultGen for Cuboid<V> {
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

impl<P> DefaultGen for Segment<P>
    where P: Point {
    fn generate<R: Rng>(rng: &mut R) -> Segment<P> {
        Segment::new(na::orig::<P>() + rng.gen(), na::orig::<P>() + rng.gen())
    }
}

impl<P> DefaultGen for Triangle<P>
    where P: Point {
    fn generate<R: Rng>(rng: &mut R) -> Triangle<P> {
        Triangle::new(na::orig::<P>() + rng.gen(), na::orig::<P>() + rng.gen(), na::orig::<P>() + rng.gen())
    }
}

impl<P> DefaultGen for ConvexHull<P>
    where P: Point {
    fn generate<R: Rng>(rng: &mut R) -> ConvexHull<P> {
        // It is recommanded to have at most 100 points.
        // Otherwise, a smarter structure like the DK hierarchy would be needed.
        let pts = (0 .. 100).map(|_| na::orig::<P>() + rng.gen::<P::Vect>()).collect();
        ConvexHull::new(pts)
    }
}

impl<P> DefaultGen for Ray<P>
    where P: Point {
    fn generate<R: Rng>(rng: &mut R) -> Ray<P> {
        // The generate ray will always point to the origin.
        let shift = rng.gen::<P::Vect>() * na::cast(10.0f64);
        Ray::new(na::orig::<P>() + shift, -shift)
    }
}

impl<P> DefaultGen for AABB<P>
    where P: Point {
    fn generate<R: Rng>(rng: &mut R) -> AABB<P> {
        // an AABB centered at the origin.
        let half_extents = na::abs(&rng.gen::<P::Vect>());
        AABB::new(na::orig::<P>() + (-half_extents), na::orig::<P>() + half_extents)
    }
}

impl<P> DefaultGen for BoundingSphere<P>
    where P: Point {
    fn generate<R: Rng>(rng: &mut R) -> BoundingSphere<P> {
        // a bounding sphere centered at the origin.
        BoundingSphere::new(na::orig(), na::abs(&rng.gen::<<P::Vect as Vect>::Scalar>()))
    }
}
