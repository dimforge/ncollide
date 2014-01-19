//!
//! Geometry composed from the union of primitives.
//!

use nalgebra::na;
use bounding_volume::{LooseBoundingVolume, AABB, HasAABB};
use ray::Ray;
use partitioning::BVT;
use partitioning::{BoundingVolumeInterferencesCollector, RayInterferencesCollector};
use geom::{Geom, ConcaveGeom};
use math::M;

/// A compound geometry with an aabb bounding volume.
///
/// A compound geometry is a geometry composed of the union of several simpler geometry. This is
/// the main way of creating a concave geometry from convex parts. Each parts can have its own
/// delta transformation to shift or rotate it with regard to the other geometries.
pub struct Compound {
    priv shapes: ~[(M, ~Geom)],
    priv bvt:    BVT<uint, AABB>,
    priv bvs:    ~[AABB]
}

impl Clone for Compound {
    fn clone(&self) -> Compound {
        Compound {
            shapes: self.shapes.map(|&(ref m, ref s)| (m.clone(), s.duplicate())),
            bvt:    self.bvt.clone(),
            bvs:    self.bvs.clone()
        }
    }
}

impl Compound {
    /// Builds a new compound shape from a list of shape with their respective delta
    /// transformation.
    pub fn new(shapes: ~[(M, ~Geom)]) -> Compound {
        let mut bvs    = ~[];
        let mut leaves = ~[];

        for (i, &(ref delta, ref shape)) in shapes.iter().enumerate() {
            let bv = shape.aabb(delta).loosened(na::cast(0.04)); // loosen for better persistancy

            bvs.push(bv.clone());
            leaves.push((i, bv));
        }

        let bvt = BVT::new_kdtree(leaves);

        Compound {
            shapes: shapes,
            bvt:    bvt,
            bvs:    bvs
        }
    }
}

impl Compound {
    /// The shapes of this compound geometry.
    #[inline]
    pub fn shapes<'r>(&'r self) -> &'r [(M, ~Geom)] {
        let res: &'r [(M, ~Geom)] = self.shapes;

        res
    }

    /// The optimization structure used by this compound geometry.
    #[inline]
    pub fn bvt<'r>(&'r self) -> &'r BVT<uint, AABB> {
        &'r self.bvt
    }

    /// The shapes bounding volumes.
    #[inline]
    pub fn bounding_volumes<'r>(&'r self) -> &'r [AABB] {
        let res: &'r [AABB] = self.bvs;

        res
    }
}

impl ConcaveGeom for Compound {
    #[inline(always)]
    fn map_part_at(&self, i: uint, f: |&M, &Geom| -> ()) {
        let (ref m, ref g) = self.shapes[i];

        f(m, *g)
    }

    #[inline(always)]
    fn map_transformed_part_at(&self, m: &M, i: uint, f: |&M, &Geom| -> ()) {
        let (ref lm, ref g) = self.shapes[i];

        f(&(m * *lm), *g)
    }

    #[inline]
    fn approx_interferences_with_aabb(&self, aabb: &AABB, out: &mut ~[uint]) {
        let mut visitor = BoundingVolumeInterferencesCollector::new(aabb, out);
        self.bvt.visit(&mut visitor);
    }

    #[inline]
    fn approx_interferences_with_ray(&self, ray: &Ray, out: &mut ~[uint]) {
        let mut visitor = RayInterferencesCollector::new(ray, out);
        self.bvt.visit(&mut visitor);
    }

    #[inline]
    fn aabb_at<'a>(&'a self, i: uint) -> &'a AABB {
        &'a self.bvs[i]
    }
}
