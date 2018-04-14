use rand::Rng;
use na;
use na::{Id, Isometry2, Isometry3, Mat2, Mat3, Mat4, Point2, Point3, Point4, Vec4, Vector2,
         Vector3};
use ncollide::bounding_volume::{BoundingSphere, AABB};
use ncollide::shape::{Ball, Capsule, Cone, ConvexHull, Cuboid, Cylinder, Segment, Triangle};
use ncollide::math::{Point, Scalar, Vector};
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

impl<N: Real> DefaultGen for Cuboid<V> {
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

impl<P> DefaultGen for Segment<N>
where
    N: Real,
{
    fn generate<R: Rng>(rng: &mut R) -> Segment<N> {
        Segment::new(Point::origin() + rng.gen(), Point::origin() + rng.gen())
    }
}

impl<P> DefaultGen for Triangle<N>
where
    N: Real,
{
    fn generate<R: Rng>(rng: &mut R) -> Triangle<N> {
        Triangle::new(
            Point::origin() + rng.gen(),
            Point::origin() + rng.gen(),
            Point::origin() + rng.gen(),
        )
    }
}

impl<P> DefaultGen for ConvexHull<N>
where
    N: Real,
{
    fn generate<R: Rng>(rng: &mut R) -> ConvexHull<N> {
        // It is recommanded to have at most 100 points.
        // Otherwise, a smarter structure like the DK hierarchy would be needed.
        let pts = (0..100)
            .map(|_| Point::origin() + rng.gen::<Vector<N>>())
            .collect();
        ConvexHull::new(pts)
    }
}

impl<P> DefaultGen for Ray<N>
where
    N: Real,
{
    fn generate<R: Rng>(rng: &mut R) -> Ray<N> {
        // The generate ray will always point to the origin.
        let shift = rng.gen::<Vector<N>>() * na::convert(10.0f64);
        Ray::new(Point::origin() + shift, -shift)
    }
}

impl<P> DefaultGen for AABB<N>
where
    N: Real,
{
    fn generate<R: Rng>(rng: &mut R) -> AABB<N> {
        // an AABB centered at the origin.
        let half_extents = na::abs(&rng.gen::<Vector<N>>());
        AABB::new(Point::origin() + (-half_extents), Point::origin() + half_extents)
    }
}

impl<P> DefaultGen for BoundingSphere<N>
where
    N: Real,
{
    fn generate<R: Rng>(rng: &mut R) -> BoundingSphere<N> {
        // a bounding sphere centered at the origin.
        BoundingSphere::new(na::origin(), na::abs(&rng.gen::<N>()))
    }
}
