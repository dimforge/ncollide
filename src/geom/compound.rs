//!
//! Geometry composed from the union of primitives.
//!

use std::num::Zero;
use nalgebra::na;
use nalgebra::na::{AlgebraicVecExt, Cast, Translation, AbsoluteRotate, Transform, Rotate};
use bounding_volume::{LooseBoundingVolume, AABB, HasAABB};
use volumetric::InertiaTensor;
use ray::Ray;
use partitioning::bvt::BVT;
use partitioning::bvt_visitor::{BoundingVolumeInterferencesCollector, RayInterferencesCollector};
use partitioning::bvt;
use geom::{Geom, ConcaveGeom};

/// A compound geometry with an aabb bounding volume. A compound geometry is a geometry composed of
/// the union of several simpler geometry. This is the main way of creating a concave geometry from
/// convex parts. Each parts can have its own delta transformation to shift or rotate it with
/// regard to the other geometries.
pub struct Compound<N, V, M, II> {
    priv shapes: ~[(M, ~Geom<N, V, M, II>)],
    priv bvt:    BVT<uint, AABB<N, V>>,
    priv bvs:    ~[AABB<N, V>]
}

impl<N: Clone, V: Clone, M: Clone, II> Clone for Compound<N, V, M, II> {
    fn clone(&self) -> Compound<N, V, M, II> {
        Compound {
            shapes: self.shapes.map(|&(ref m, ref s)| (m.clone(), s.duplicate())),
            bvt:    self.bvt.clone(),
            bvs:    self.bvs.clone()
        }
    }
}

impl<N: 'static + Algebraic + Primitive + Orderable + Signed + Cast<f32> + Clone,
     V: 'static + AlgebraicVecExt<N> + Clone,
     M,
     II>
Compound<N, V, M, II> {
    /// Builds a new compound shape from a list of shape with their respective delta
    /// transformation.
    pub fn new(shapes: ~[(M, ~Geom<N, V, M, II>)]) -> Compound<N, V, M, II> {
        let mut bvs    = ~[];
        let mut leaves = ~[];

        for (i, &(ref delta, ref shape)) in shapes.iter().enumerate() {
            let bv = shape.aabb(delta).loosened(na::cast(0.04)); // loosen for better persistancy

            bvs.push(bv.clone());
            leaves.push((i, bv));
        }

        let bvt = BVT::new_with_partitioner(leaves, bvt::kdtree_partitioner);

        Compound {
            shapes: shapes,
            bvt:    bvt,
            bvs:    bvs
        }
    }
}

impl<N, V, M, II> Compound<N, V, M, II> {
    /// The shapes of this compound geometry.
    #[inline]
    pub fn shapes<'r>(&'r self) -> &'r [(M, ~Geom<N, V, M, II>)] {
        let res: &'r [(M, ~Geom<N, V, M, II>)] = self.shapes;

        res
    }

    /// The optimization structure used by this compound geometry.
    #[inline]
    pub fn bvt<'r>(&'r self) -> &'r BVT<uint, AABB<N, V>> {
        &'r self.bvt
    }

    /// The shapes bounding volumes.
    #[inline]
    pub fn bounding_volumes<'r>(&'r self) -> &'r [AABB<N, V>] {
        let res: &'r [AABB<N, V>] = self.bvs;

        res
    }
}

impl<N:  Clone + Zero + Num + Primitive + Orderable + Cast<f32> + Algebraic,
     LV: Clone + Zero + AlgebraicVecExt<N>,
     AV,
     M:  Clone + Mul<M, M> + Translation<LV> + AbsoluteRotate<LV> + Transform<LV> + Rotate<LV>,
     II: Zero + Add<II, II> + InertiaTensor<N, LV, AV, M>>
ConcaveGeom<N, LV, M, II> for Compound<N, LV, M, II> {
    #[inline(always)]
    fn map_part_at(&self, i: uint, f: |&M, &Geom<N, LV, M, II>| -> ()) {
        let (ref m, ref g) = self.shapes[i];

        f(m, *g)
    }

    #[inline(always)]
    fn map_transformed_part_at(&self, m: &M, i: uint, f: |&M, &Geom<N, LV, M, II>| -> ()) {
        let (ref lm, ref g) = self.shapes[i];

        f(&(m * *lm), *g)
    }

    #[inline]
    fn approx_interferences_with_aabb(&self, aabb: &AABB<N, LV>, out: &mut ~[uint]) {
        let mut visitor = BoundingVolumeInterferencesCollector::new(aabb, out);
        self.bvt.visit(&mut visitor);
    }

    #[inline]
    fn approx_interferences_with_ray(&self, ray: &Ray<LV>, out: &mut ~[uint]) {
        let mut visitor = RayInterferencesCollector::new(ray, out);
        self.bvt.visit(&mut visitor);
    }

    #[inline]
    fn aabb_at<'a>(&'a self, i: uint) -> &'a AABB<N, LV> {
        &'a self.bvs[i]
    }
}
