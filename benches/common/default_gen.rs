use rand::Rng;
use na;
use na::{Vector2, Vector3, Vec4, Point2, Point3, Point4, Mat2, Mat3, Mat4, Isometry2, Isometry3, Id};
use ncollide::bounding_volume::{AABB, BoundingSphere};
use ncollide::shape::{Ball, Cuboid, Capsule, Cone, Cylinder, Segment, Triangle, ConvexHull};
use ncollide::math::{Scalar, Point, Vector};
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

impl_rand_default_gen!(Vector2<f32>);
impl_rand_default_gen!(Vector3<f32>);
impl_rand_default_gen!(Vec4<f32>);
impl_rand_default_gen!(Point2<f32>);
impl_rand_default_gen!(Point3<f32>);
impl_rand_default_gen!(Point4<f32>);
impl_rand_default_gen!(Mat2<f32>);
impl_rand_default_gen!(Mat3<f32>);
impl_rand_default_gen!(Mat4<f32>);
impl_rand_default_gen!(Isometry2<f32>);
impl_rand_default_gen!(Isometry3<f32>);
impl_rand_default_gen!(Vector2<f64>);
impl_rand_default_gen!(Vector3<f64>);
impl_rand_default_gen!(Vec4<f64>);
impl_rand_default_gen!(Point2<f64>);
impl_rand_default_gen!(Point3<f64>);
impl_rand_default_gen!(Point4<f64>);
impl_rand_default_gen!(Mat2<f64>);
impl_rand_default_gen!(Mat3<f64>);
impl_rand_default_gen!(Mat4<f64>);
impl_rand_default_gen!(Isometry2<f64>);
impl_rand_default_gen!(Isometry3<f64>);
impl_rand_default_gen!(f32);
impl_rand_default_gen!(f64);
impl_rand_default_gen!(bool);

impl DefaultGen for Id {
    fn generate<R: Rng>(_: &mut R) -> Id {
        Id::new()
    }
}

impl<N: Scalar> DefaultGen for Ball<N> {
    fn generate<R: Rng>(rng: &mut R) -> Ball<N> {
        Ball::new(na::abs(&rng.gen::<N>()))
    }
}

impl<V: Vector> DefaultGen for Cuboid<V> {
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
        Segment::new(P::origin() + rng.gen(), P::origin() + rng.gen())
    }
}

impl<P> DefaultGen for Triangle<P>
    where P: Point {
    fn generate<R: Rng>(rng: &mut R) -> Triangle<P> {
        Triangle::new(P::origin() + rng.gen(), P::origin() + rng.gen(), P::origin() + rng.gen())
    }
}

impl<P> DefaultGen for ConvexHull<P>
    where P: Point {
    fn generate<R: Rng>(rng: &mut R) -> ConvexHull<P> {
        // It is recommanded to have at most 100 points.
        // Otherwise, a smarter structure like the DK hierarchy would be needed.
        let pts = (0 .. 100).map(|_| P::origin() + rng.gen::<P::Vector>()).collect();
        ConvexHull::new(pts)
    }
}

impl<P> DefaultGen for Ray<P>
    where P: Point {
    fn generate<R: Rng>(rng: &mut R) -> Ray<P> {
        // The generate ray will always point to the origin.
        let shift = rng.gen::<P::Vector>() * na::convert(10.0f64);
        Ray::new(P::origin() + shift, -shift)
    }
}

impl<P> DefaultGen for AABB<P>
    where P: Point {
    fn generate<R: Rng>(rng: &mut R) -> AABB<P> {
        // an AABB centered at the origin.
        let half_extents = na::abs(&rng.gen::<P::Vector>());
        AABB::new(P::origin() + (-half_extents), P::origin() + half_extents)
    }
}

impl<P> DefaultGen for BoundingSphere<P>
    where P: Point {
    fn generate<R: Rng>(rng: &mut R) -> BoundingSphere<P> {
        // a bounding sphere centered at the origin.
        BoundingSphere::new(na::origin(), na::abs(&rng.gen::<P::Real>()))
    }
}
