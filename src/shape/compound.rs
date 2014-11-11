//!
//! Shapeetry composed from the union of primitives.
//!

use std::num::Zero;
use std::sync::Arc;
use na::{Translate, Cross, AbsoluteRotate, Rotate, Transform};
use na;
use bounding_volume::{AABB, HasAABB, BoundingVolume};
use volumetric::{Volumetric, InertiaTensor};
use partitioning::BVT;
use shape::{Shape, ConcaveShape};
use math::{Scalar, Point, Vect};


/// Structure used to build a `Compound` shape.
///
/// This accumulates the geometries and their volumetric properties.
#[deriving(Clone)]
pub struct CompoundData<N, P, V, M, I> {
    geoms: Vec<(M, Arc<Box<Shape<N, P, V, M> + Send + Sync>>)>,
    props: Vec<(N, N, P, I)>
}

impl<N, P, V, M, I> CompoundData<N, P, V, M, I>
    where N: Scalar,
          I: Mul<N, I> {
    /// Creates a new `CompoundData`.
    pub fn new() -> CompoundData<N, P, V, M, I> {
        CompoundData {
            geoms: Vec::new(),
            props: Vec::new()
        }
    }

    /// Adds a new shape; its mass properties are automatically computed.
    #[inline]
    pub fn push_geom<S>(&mut self, delta: M, geom: S, density: N)
        where S: Shape<N, P, V, M> + Volumetric<N, P, I> + Send + Sync {
        let (m, c, i) = geom.mass_properties(density);
        let s         = geom.surface();

        self.push_geom_with_mass_properties(delta, geom, (s, m, c, i))
    }

    /// Adds a new shape with the given mass properties.
    #[inline]
    pub fn push_geom_with_mass_properties<S>(&mut self, delta: M, geom:  S, props: (N, N, P, I))
        where S: Shape<N, P, V, M> + Send + Sync {
        self.push_shared_geom_with_mass_properties(
            delta,
            Arc::new(box geom as Box<Shape<N, P, V, M> + Send + Sync>),
            props)
    }

    /// Adds a new shared shape with the given mass properties.
    #[inline]
    pub fn push_shared_geom_with_mass_properties(&mut self,
                                                 delta: M,
                                                 geom:  Arc<Box<Shape<N, P, V, M> + Send + Sync>>,
                                                 props: (N, N, P, I)) {
        self.geoms.push((delta, geom));
        self.props.push(props);
    }

    /// The geometries stored by this `CompoundData`.
    #[inline]
    pub fn geoms(&self) -> &[(M, Arc<Box<Shape<N, P, V, M> + Send + Sync>>)] {
        self.geoms.as_slice()
    }

    // FIXME: this is not a very good name.
    /// The geometries stored by this `CompoundData`.
    #[inline]
    pub fn mass_properties_list(&self) -> &[(N, N, P, I)] {
        self.props.as_slice()
    }
}

/// A compound shape with an aabb bounding volume.
///
/// A compound shape is a shape composed of the union of several simpler shape. This is
/// the main way of creating a concave shape from convex parts. Each parts can have its own
/// delta transformation to shift or rotate it with regard to the other geometries.
#[deriving(Clone)]
pub struct Compound<N, P, V, M, I> {
    surface: N,
    mass:    N,
    inertia: I,
    com:     P,
    geoms:   Vec<(M, Arc<Box<Shape<N, P, V, M> + Send + Sync>>)>,
    bvt:     BVT<uint, AABB<P>>,
    bvs:     Vec<AABB<P>>
}

impl<N, P, V, AV, M, I> Compound<N, P, V, M, I>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P> + Cross<AV>,
          M: Mul<P, P>,
          I: Zero + Add<I, I> + Mul<N, I> + InertiaTensor<N, P, AV, M> {
    /// Builds a new compound shape.
    pub fn new(data: CompoundData<N, P, V, M, I>) -> Compound<N, P, V, M, I> {
        let mut bvs    = Vec::new();
        let mut leaves = Vec::new();

        for (i, &(ref delta, ref geom)) in data.geoms().iter().enumerate() {
            let bv = geom.aabb(delta).loosened(na::cast(0.04f64)); // loosen for better persistancy

            bvs.push(bv.clone());
            leaves.push((i, bv));
        }

        let bvt = BVT::new_balanced(leaves);

        let (mass, com, inertia) = data.mass_properties(na::one());
        let surface = data.surface();

        Compound {
            surface: surface,
            mass:    mass,
            inertia: inertia,
            com:     com,
            geoms:   data.geoms,
            bvt:     bvt,
            bvs:     bvs
        }
    }
}

impl<N, P, V, M, I> Compound<N, P, V, M, I>
    where N: Clone {
    /// The geometries of this compound shape.
    #[inline]
    pub fn geoms(&self) -> &[(M, Arc<Box<Shape<N, P, V, M> + Send + Sync>>)] {
        self.geoms.as_slice()
    }

    /// The optimization structure used by this compound shape.
    #[inline]
    pub fn bvt(&self) -> &BVT<uint, AABB<P>> {
        &self.bvt
    }

    /// The geometries bounding volumes.
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
    pub fn angular_inertia(&self) -> &I {
        &self.inertia
    }

    #[doc(hidden)]
    #[inline]
    pub fn center_of_mass(&self) -> &P {
        &self.com
    }
}

impl<N, P, V, M, I> ConcaveShape<N, P, V, M> for Compound<N, P, V, M, I>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Send + Sync + AbsoluteRotate<V> + Transform<P> + Rotate<V> + Mul<M, M> + Clone,
          I: Send + Sync + Clone {
    #[inline(always)]
    fn map_part_at<T>(&self, i: uint, f: |&M, &Shape<N, P, V, M>| -> T) -> T{
        let &(ref m, ref g) = &self.geoms[i];

        f(m, &***g)
    }

    #[inline(always)]
    fn map_transformed_part_at<T>(&self, m: &M, i: uint, f: |&M, &Shape<N, P, V, M>| -> T) -> T{
        let &(ref lm, ref g) = &self.geoms[i];

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
