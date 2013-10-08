use std::num::One;
use nalgebra::na::{Translation, Rotate, Transform, AbsoluteRotate, AlgebraicVecExt};
use bounding_volume::{HasAABB, AABB};
use geom::{Plane, Ball, Box, Cone, Cylinder, Capsule, Implicit, HasMargin, CompoundAABB};
use ray::{Ray, RayCast, RayCastWithTransform};

/// Enumeration grouping all common shapes. Used to simplify collision detection
/// dispatch.
#[deriving(Clone, Encodable, Decodable)]
pub enum Geom<N, V, M> { // FIXME: rename that
    /// A plane geometry.
    PlaneGeom(Plane<N, V>),
    /// A compound geometry.
    CompoundGeom(@CompoundAABB<N, V, M, Geom<N, V, M>>),
    /// Geometry describable with a support function.
    ImplicitGeom(IGeom<N, V, M>)
}

/// Geometries with a support function.
#[deriving(Clone, Encodable, Decodable)]
pub enum IGeom<N, V, M> {
    /// A ball geometry.
    BallGeom(Ball<N>),
    /// A box geometry.
    BoxGeom(Box<N, V>),
    /// A cone geometry.
    ConeGeom(Cone<N>),
    /// A cylinder geometry.
    CylinderGeom(Cylinder<N>),
    /// A capsule geometry.
    CapsuleGeom(Capsule<N>)
}

impl<N: One + ToStr, V: ToStr, M> Geom<N, V, M> {
    /// Creates a new `Geom` from a plane.
    #[inline]
    pub fn new_plane(p: Plane<N, V>) -> Geom<N, V, M> {
        PlaneGeom(p)
    }

    /// Creates a new `Geom` from a compound geometry.
    #[inline]
    pub fn new_compound(c: @CompoundAABB<N, V, M, Geom<N, V, M>>)
        -> Geom<N, V, M> {
        CompoundGeom(c)
    }

    /// Creates a new `Geom` from a ball.
    #[inline]
    pub fn new_ball(b: Ball<N>) -> Geom<N, V, M> {
        ImplicitGeom(BallGeom(b))
    }

    /// Creates a new `Geom` from a cylinder.
    #[inline]
    pub fn new_cylinder(b: Cylinder<N>) -> Geom<N, V, M> {
        ImplicitGeom(CylinderGeom(b))
    }

    /// Creates a new `Geom` from a box.
    #[inline]
    pub fn new_box(b: Box<N, V>) -> Geom<N, V, M> {
        ImplicitGeom(BoxGeom(b))
    }

    /// Creates a new `Geom` from a cone.
    #[inline]
    pub fn new_cone(b: Cone<N>) -> Geom<N, V, M> {
        ImplicitGeom(ConeGeom(b))
    }

}

impl<N, V, M> Geom<N, V, M> {
    /**
     * Convenience method to extract a plane from the enumation. Fails if the
     * pattern `Plane(_)` is not matched.
     */
    #[inline]
    pub fn plane<'r>(&'r self) -> &'r Plane<N, V> {
        match *self {
            PlaneGeom(ref p) => p,
            _ => fail!("Unexpected geometry: this is not a plane.")
        }
    }

    /**
     * Convenience method to extract a compound geometry from the enumation. Fails if the
     * pattern `Compound(_)` is not matched.
     */
    #[inline]
    pub fn compound(&self) -> @CompoundAABB<N, V, M, Geom<N, V, M>> {
        match *self {
            CompoundGeom(c) => c,
            _ => fail!("Unexpected geometry: this is not a compound.")
        }
    }

    /**
     * Convenience method to extract an implicit geometry from the enumation. Fails if the
     * pattern `Implicit(_)` is not matched.
     */
    #[inline]
    pub fn implicit<'r>(&'r self) -> &'r IGeom<N, V, M> {
        match *self {
            ImplicitGeom(ref i) => i,
            _ => fail!("Unexpected geometry: this is not an implicit.")
        }
    }

    /**
     * Convenience method to extract a ball from the enumation. Fails if the
     * pattern `Implicit(Ball(_))` is not matched.
     */
    #[inline]
    pub fn ball<'r>(&'r self) -> &'r Ball<N> {
        match *self {
            ImplicitGeom(BallGeom(ref b)) => b,
            _ => fail!("Unexpected geometry: this is not a ball.")
        }
    }

    /**
     * Convenience method to extract a cone from the enumation. Fails if the
     * pattern `Implicit(Cone(_))` is not matched.
     */
    #[inline]
    pub fn cone<'r>(&'r self) -> &'r Cone<N> {
        match *self {
            ImplicitGeom(ConeGeom(ref c)) => c,
            _ => fail!("Unexpected geometry: this is not a ball.")
        }
    }

    /**
     * Convenience method to extract a cylinder from the enumation. Fails if the
     * pattern `Implicit(Cylinder(_))` is not matched.
     */
    #[inline]
    pub fn cylinder<'r>(&'r self) -> &'r Cylinder<N> {
        match *self {
            ImplicitGeom(CylinderGeom(ref c)) => c,
            _ => fail!("Unexpected geometry: this is not a ball.")
        }
    }

    /**
     * Convenience method to extract a box from the enumation. Fails if the
     * pattern `Implicit(Box(_))` is not matched.
     */
    #[inline]
    pub fn box<'r>(&'r self) -> &'r Box<N, V> {
        match *self {
            ImplicitGeom(BoxGeom(ref b)) => b,
            _ => fail!("Unexpected geometry: this is not a ball.")
        }
    }
}

impl<N: FromPrimitive + Primitive + Orderable + Algebraic + Signed + Clone + ToStr,
     V: AlgebraicVecExt<N> + Clone + ToStr,
     M: Translation<V> + Rotate<V> + Transform<V> + Mul<M, M> + AbsoluteRotate<V>>
HasAABB<N, V, M> for Geom<N, V, M> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<N, V> {
        match *self {
            PlaneGeom(ref p)    => p.aabb(m),
            CompoundGeom(ref c) => c.aabb(m),
            ImplicitGeom(ref i) => {
                match *i {
                    BallGeom(ref b)     => b.aabb(m),
                    BoxGeom(ref b)      => b.aabb(m),
                    ConeGeom(ref c)     => c.aabb(m),
                    CylinderGeom(ref c) => c.aabb(m),
                    CapsuleGeom(ref c)  => c.aabb(m)
                }
            }
        }
    }
}

// FIXME: move this to the ray folder?
impl<N: Algebraic + Bounded + Orderable + Primitive + Float + FromPrimitive + Clone + ToStr,
     V: 'static + AlgebraicVecExt<N> + Clone + ToStr,
     M: Rotate<V> + Transform<V>>
RayCast<N, V> for Geom<N, V, M> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        match *self {
            PlaneGeom(ref p)    => p.toi_with_ray(ray),
            CompoundGeom(ref c) => c.toi_with_ray(ray),
            ImplicitGeom(ref i) => {
                match *i {
                    BallGeom(ref b)     => b.toi_with_ray(ray),
                    BoxGeom(ref b)      => b.toi_with_ray(ray),
                    ConeGeom(ref c)     => c.toi_with_ray(ray),
                    CylinderGeom(ref c) => c.toi_with_ray(ray),
                    CapsuleGeom(ref c)  => c.toi_with_ray(ray),
                }
            }
        }
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        match *self {
            PlaneGeom(ref p)    => p.toi_and_normal_with_ray(ray),
            CompoundGeom(ref c) => c.toi_and_normal_with_ray(ray),
            ImplicitGeom(ref i) => {
                match *i {
                    BallGeom(ref b)     => b.toi_and_normal_with_ray(ray),
                    BoxGeom(ref b)      => b.toi_and_normal_with_ray(ray),
                    ConeGeom(ref c)     => c.toi_and_normal_with_ray(ray),
                    CylinderGeom(ref c) => c.toi_and_normal_with_ray(ray),
                    CapsuleGeom(ref c)  => c.toi_and_normal_with_ray(ray),
                }
            }
        }
    }
}

impl<N: Algebraic + Bounded + Orderable + Primitive + Float + FromPrimitive + Clone + ToStr,
     V: 'static + AlgebraicVecExt<N> + Clone + ToStr,
     M: Rotate<V> + Transform<V>>
RayCastWithTransform<N, V, M> for Geom<N, V, M> { }

impl<N: Clone + Add<N, N>, V, M> HasMargin<N> for IGeom<N, V, M> {
    #[inline]
    fn margin(&self) -> N {
        match *self {
            BallGeom(ref b)     => b.margin(),
            BoxGeom(ref b)      => b.margin(),
            ConeGeom(ref c)     => c.margin(),
            CylinderGeom(ref c) => c.margin(),
            CapsuleGeom(ref c)  => c.margin()
        }
    }
}

impl<N: Algebraic + Signed + Orderable + Clone,
     V: AlgebraicVecExt<N> + Clone,
     M: Translation<V> + Rotate<V> + Transform<V>>
Implicit<N, V, M> for IGeom<N, V, M> {
    #[inline]
    fn support_point_without_margin(&self, transform: &M, dir: &V) -> V {
        match *self {
            BallGeom(ref b)     => b.support_point_without_margin(transform, dir),
            BoxGeom(ref b)      => b.support_point_without_margin(transform, dir),
            ConeGeom(ref c)     => c.support_point_without_margin(transform, dir),
            CylinderGeom(ref c) => c.support_point_without_margin(transform, dir),
            CapsuleGeom(ref c)  => c.support_point_without_margin(transform, dir)
        }
    }

    #[inline]
    fn support_point(&self, transform: &M, dir: &V) -> V {
        match *self {
            BallGeom(ref b)     => b.support_point(transform, dir),
            BoxGeom(ref b)      => b.support_point(transform, dir),
            ConeGeom(ref c)     => c.support_point(transform, dir),
            CylinderGeom(ref c) => c.support_point(transform, dir),
            CapsuleGeom(ref c)  => c.support_point(transform, dir)
        }
    }
}
