use na::{Point2, Point3, Point4, Vector2, Vector3, Vec4, Transform, Rotate, Translation};
use support_map::SupportMap;

use shape::{Ball2, Ball3, Ball4,
           Cuboid2, Cuboid3, Cuboid4,
           Capsule2, Capsule3, Capsule4,
           Cone2, Cone3, Cone4,
           Cylinder2, Cylinder3, Cylinder4,
           Segment2, Segment3, Segment4,
           Triangle2, Triangle3, Triangle4};

macro_rules! specialize_implicit_impl(
    ($t: ident, $p: ident, $v: ident, $n: ident) => (
        impl<M> SupportMap<$p<$n>, $v<$n>, M> for $t
            where M: Transform<$p<$n>> + Rotate<$v<$n>> + Translation<$v<$n>> {
            #[inline]
            fn support_point(&self, transform: &M, dir: &$v<$n>) -> $p<$n> {
                self.as_ref().support_point(transform, dir)
            }
        }
    )
)

specialize_implicit_impl!(Ball2, Point2, Vector2, f32)
specialize_implicit_impl!(Cuboid2, Point2, Vector2, f32)
specialize_implicit_impl!(Capsule2, Point2, Vector2, f32)
specialize_implicit_impl!(Cone2, Point2, Vector2, f32)
specialize_implicit_impl!(Cylinder2, Point2, Vector2, f32)
specialize_implicit_impl!(Segment2, Point2, Vector2, f32)
specialize_implicit_impl!(Triangle2, Point2, Vector2, f32)

specialize_implicit_impl!(Ball3, Point3, Vector3, f32)
specialize_implicit_impl!(Cuboid3, Point3, Vector3, f32)
specialize_implicit_impl!(Capsule3, Point3, Vector3, f32)
specialize_implicit_impl!(Cone3, Point3, Vector3, f32)
specialize_implicit_impl!(Cylinder3, Point3, Vector3, f32)
specialize_implicit_impl!(Segment3, Point3, Vector3, f32)
specialize_implicit_impl!(Triangle3, Point3, Vector3, f32)

specialize_implicit_impl!(Ball4, Point4, Vec4, f32)
specialize_implicit_impl!(Cuboid4, Point4, Vec4, f32)
specialize_implicit_impl!(Capsule4, Point4, Vec4, f32)
specialize_implicit_impl!(Cone4, Point4, Vec4, f32)
specialize_implicit_impl!(Cylinder4, Point4, Vec4, f32)
specialize_implicit_impl!(Segment4, Point4, Vec4, f32)
specialize_implicit_impl!(Triangle4, Point4, Vec4, f32)
