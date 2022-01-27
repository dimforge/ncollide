use na::{
    self, Isometry2, Isometry3, Matrix2, Matrix3, Matrix4, Point2, Point3, Point4, RealField,
    Vector2, Vector3, Vector4,
};
use ncollide3d::bounding_volume::{BoundingSphere, AABB};
use ncollide3d::query::Ray;
use ncollide3d::shape::{Ball, Capsule, Cone, ConvexHull, Cuboid, Cylinder, Segment, Triangle};
use rand::distributions::{Distribution, Standard};
use rand::Rng;

pub trait DefaultGen {
    fn generate<R: Rng>(rng: &mut R) -> Self;
}

pub fn generate<T: DefaultGen, R: Rng>(rng: &mut R) -> T {
    DefaultGen::generate(rng)
}

macro_rules! impl_rand_default_gen (
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
impl_rand_default_gen!(Vector4<f32>);
impl_rand_default_gen!(Point2<f32>);
impl_rand_default_gen!(Point3<f32>);
impl_rand_default_gen!(Point4<f32>);
impl_rand_default_gen!(Matrix2<f32>);
impl_rand_default_gen!(Matrix3<f32>);
impl_rand_default_gen!(Matrix4<f32>);
impl_rand_default_gen!(Isometry2<f32>);
impl_rand_default_gen!(Isometry3<f32>);
impl_rand_default_gen!(Vector2<f64>);
impl_rand_default_gen!(Vector3<f64>);
impl_rand_default_gen!(Vector4<f64>);
impl_rand_default_gen!(Point2<f64>);
impl_rand_default_gen!(Point3<f64>);
impl_rand_default_gen!(Point4<f64>);
impl_rand_default_gen!(Matrix2<f64>);
impl_rand_default_gen!(Matrix3<f64>);
impl_rand_default_gen!(Matrix4<f64>);
impl_rand_default_gen!(Isometry2<f64>);
impl_rand_default_gen!(Isometry3<f64>);
impl_rand_default_gen!(f32);
impl_rand_default_gen!(f64);
impl_rand_default_gen!(bool);

impl<N: RealField + Copy> DefaultGen for Ball<N>
where
    Standard: Distribution<N>,
{
    fn generate<R: Rng>(rng: &mut R) -> Ball<N> {
        Ball::new(rng.gen::<N>().abs())
    }
}

impl<N: RealField + Copy> DefaultGen for Cuboid<N>
where
    Standard: Distribution<Vector3<N>>,
{
    fn generate<R: Rng>(rng: &mut R) -> Cuboid<N> {
        Cuboid::new(rng.gen::<Vector3<N>>().abs())
    }
}

impl<N: RealField + Copy> DefaultGen for Capsule<N>
where
    Standard: Distribution<N>,
{
    fn generate<R: Rng>(rng: &mut R) -> Capsule<N> {
        Capsule::new(rng.gen::<N>().abs(), rng.gen::<N>().abs())
    }
}

impl<N: RealField + Copy> DefaultGen for Cone<N>
where
    Standard: Distribution<N>,
{
    fn generate<R: Rng>(rng: &mut R) -> Cone<N> {
        Cone::new(rng.gen::<N>().abs(), rng.gen::<N>().abs())
    }
}

impl<N: RealField + Copy> DefaultGen for Cylinder<N>
where
    Standard: Distribution<N>,
{
    fn generate<R: Rng>(rng: &mut R) -> Cylinder<N> {
        Cylinder::new(rng.gen::<N>().abs(), rng.gen::<N>().abs())
    }
}

impl<N: RealField + Copy> DefaultGen for Segment<N>
where
    Standard: Distribution<Point3<N>>,
{
    fn generate<R: Rng>(rng: &mut R) -> Segment<N> {
        Segment::new(rng.gen(), rng.gen())
    }
}

impl<N: RealField + Copy> DefaultGen for Triangle<N>
where
    Standard: Distribution<Point3<N>>,
{
    fn generate<R: Rng>(rng: &mut R) -> Triangle<N> {
        Triangle::new(rng.gen(), rng.gen(), rng.gen())
    }
}

impl<N: RealField + Copy> DefaultGen for ConvexHull<N>
where
    Standard: Distribution<Point3<N>>,
{
    fn generate<R: Rng>(rng: &mut R) -> ConvexHull<N> {
        // It is recommended to have at most 100 points.
        // Otherwise, a smarter structure like the DK hierarchy would be needed.
        let pts: Vec<_> = (0..100).map(|_| rng.gen()).collect();
        ConvexHull::try_from_points(&pts).unwrap()
    }
}

impl<N: RealField + Copy> DefaultGen for Ray<N>
where
    Standard: Distribution<Vector3<N>>,
{
    fn generate<R: Rng>(rng: &mut R) -> Ray<N> {
        // The generate ray will always point to the origin.
        let shift = rng.gen::<Vector3<N>>() * na::convert::<_, N>(10.0f64);
        Ray::new(Point3::origin() + shift, -shift)
    }
}

impl<N: RealField + Copy> DefaultGen for AABB<N>
where
    Standard: Distribution<Vector3<N>>,
{
    fn generate<R: Rng>(rng: &mut R) -> AABB<N> {
        // an AABB centered at the origin.
        let half_extents = rng.gen::<Vector3<N>>().abs();
        AABB::new(
            Point3::origin() + (-half_extents),
            Point3::origin() + half_extents,
        )
    }
}

impl<N: RealField + Copy> DefaultGen for BoundingSphere<N>
where
    Standard: Distribution<N>,
{
    fn generate<R: Rng>(rng: &mut R) -> BoundingSphere<N> {
        // a bounding sphere centered at the origin.
        BoundingSphere::new(Point3::origin(), rng.gen::<N>().abs())
    }
}
