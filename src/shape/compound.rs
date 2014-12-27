//!
//! Shapeetry composed from the union of primitives.
//!

use std::any::{Any, AnyRefExt};
use std::sync::Arc;
use na::{Cross, Translate, Zero};
use na;
use bounding_volume::{AABB, HasAABB, BoundingVolume};
use volumetric::{Volumetric, InertiaTensor};
use partitioning::BVT;
use shape::{Shape, ConcaveShape};
use math::{Scalar, Point, Vect, Isometry, HasInertiaMatrix};


/// Structure used to build a `Compound` shape.
///
/// This accumulates the shapes and their volumetric properties.
#[deriving(Clone)]
pub struct CompoundData<N, P, V, M, I> {
    shapes: Vec<(M, Arc<Box<Shape<N, P, V, M> + Send + Sync>>)>,
    props:  Vec<(N, N, P, I)>
}

impl<N, P, V, M, I> CompoundData<N, P, V, M, I>
    where N: Scalar,
          I: Mul<N, I> {
    /// Creates a new `CompoundData`.
    pub fn new() -> CompoundData<N, P, V, M, I> {
        CompoundData {
            shapes: Vec::new(),
            props:  Vec::new()
        }
    }

    /// Adds a new shape; its mass properties are automatically computed.
    #[inline]
    pub fn push_shape<S>(&mut self, delta: M, shape: S, density: N)
        where S: Shape<N, P, V, M> + Volumetric<N, P, I> + Send + Sync {
        let (m, c, i) = shape.mass_properties(density);
        let s         = shape.surface();

        self.push_shape_with_mass_properties(delta, shape, (s, m, c, i))
    }

    /// Adds a new shape with the given mass properties.
    #[inline]
    pub fn push_shape_with_mass_properties<S>(&mut self, delta: M, shape:  S, props: (N, N, P, I))
        where S: Shape<N, P, V, M> + Send + Sync {
        self.push_shared_shape_with_mass_properties(
            delta,
            Arc::new(box shape as Box<Shape<N, P, V, M> + Send + Sync>),
            props)
    }

    /// Adds a new shared shape with the given mass properties.
    #[inline]
    pub fn push_shared_shape_with_mass_properties(&mut self,
                                                  delta: M,
                                                  shape:  Arc<Box<Shape<N, P, V, M> + Send + Sync>>,
                                                  props: (N, N, P, I)) {
        self.shapes.push((delta, shape));
        self.props.push(props);
    }

    /// The shapes stored by this `CompoundData`.
    #[inline]
    pub fn shapes(&self) -> &[(M, Arc<Box<Shape<N, P, V, M> + Send + Sync>>)] {
        self.shapes.as_slice()
    }

    // FIXME: this is not a very good name.
    /// The shapes stored by this `CompoundData`.
    #[inline]
    pub fn mass_properties_list(&self) -> &[(N, N, P, I)] {
        self.props.as_slice()
    }
}

/// A compound shape with an aabb bounding volume.
///
/// A compound shape is a shape composed of the union of several simpler shape. This is
/// the main way of creating a concave shape from convex parts. Each parts can have its own
/// delta transformation to shift or rotate it with regard to the other shapes.
#[deriving(Clone)]
pub struct Compound<N, P, V, M> {
    surface: N,
    mass:    N,
    // Note: this is ugly, but avoids the parametrization by I. Use associated types to avoid this?
    inertia: Arc<Box<Any + Send + Sync>>,
    com:     P,
    shapes:  Vec<(M, Arc<Box<Shape<N, P, V, M> + Send + Sync>>)>,
    bvt:     BVT<uint, AABB<P>>,
    bvs:     Vec<AABB<P>>
}

impl<N, P, V, AV, M, I> Compound<N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P> + Cross<AV>,
          M: Mul<P, P> + Copy,
          I: Send + Sync + Zero + Add<I, I> + Mul<N, I> + InertiaTensor<N, P, AV, M> + Any {
    /// Builds a new compound shape.
    pub fn new(data: CompoundData<N, P, V, M, I>) -> Compound<N, P, V, M> {
        let mut bvs    = Vec::new();
        let mut leaves = Vec::new();

        for (i, &(ref delta, ref shape)) in data.shapes().iter().enumerate() {
            let bv = shape.aabb(delta).loosened(na::cast(0.04f64)); // loosen for better persistancy

            bvs.push(bv.clone());
            leaves.push((i, bv));
        }

        let bvt = BVT::new_balanced(leaves);

        let (mass, com, inertia) = data.mass_properties(na::one());
        let surface = data.surface();

        Compound {
            surface: surface,
            mass:    mass,
            inertia: Arc::new(box inertia as Box<Any + Send + Sync>),
            com:     com,
            shapes:  data.shapes,
            bvt:     bvt,
            bvs:     bvs
        }
    }
}

impl<N, P, V, M> Compound<N, P, V, M>
    where N: Clone {
    /// The shapes of this compound shape.
    #[inline]
    pub fn shapes(&self) -> &[(M, Arc<Box<Shape<N, P, V, M> + Send + Sync>>)] {
        self.shapes.as_slice()
    }

    /// The optimization structure used by this compound shape.
    #[inline]
    pub fn bvt(&self) -> &BVT<uint, AABB<P>> {
        &self.bvt
    }

    /// The shapes bounding volumes.
    #[inline]
    pub fn bounding_volumes(&self) -> &[AABB<P>] {
        self.bvs.as_slice()
    }

    #[doc(hidden)]
    #[inline]
    pub fn surface(&self) -> N {
        self.surface.clone()
    }


    #[doc(hidden)]
    #[inline]
    pub fn mass(&self) -> N {
        self.mass.clone()
    }

    #[doc(hidden)]
    #[inline]
    pub fn center_of_mass(&self) -> &P {
        &self.com
    }
}

impl<N, P, V, M, I> Compound<N, P, V, M>
    where V: HasInertiaMatrix<I>,
          I: 'static {
    #[doc(hidden)]
    #[inline]
    pub fn angular_inertia(&self) -> &I {
        self.inertia.downcast_ref::<I>().unwrap()
    }
}

impl<N, P, V, M> ConcaveShape<N, P, V, M> for Compound<N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Isometry<N, P, V> {
    #[inline(always)]
    fn map_part_at<T>(&self, i: uint, f: |&M, &Shape<N, P, V, M>| -> T) -> T{
        let &(ref m, ref g) = &self.shapes[i];

        f(m, &***g)
    }

    #[inline(always)]
    fn map_transformed_part_at<T>(&self, m: &M, i: uint, f: |&M, &Shape<N, P, V, M>| -> T) -> T{
        let &(ref lm, ref g) = &self.shapes[i];

        f(&(*m * *lm), &***g)
    }

    #[inline]
    fn aabb_at(&self, i: uint) -> &AABB<P> {
        &self.bvs[i]
    }

    #[inline]
    fn bvt(&self) -> &BVT<uint, AABB<P>> {
        &self.bvt
    }
}
