//!
//! Shape composed from the union of primitives.
//!

use std::mem;
use bounding_volume::{BoundingVolume, AABB};
use math::Isometry;
use na::{self, Real};
use partitioning::{BVHImpl, BVT};
use shape::{CompositeShape, Shape, ShapeHandle, FeatureId};
use query::{ContactPrediction, ContactPreprocessor, Contact, ContactKinematic};

/// A compound shape with an aabb bounding volume.
///
/// A compound shape is a shape composed of the union of several simpler shape. This is
/// the main way of creating a concave shape from convex parts. Each parts can have its own
/// delta transformation to shift or rotate it with regard to the other shapes.
#[derive(Clone)]
pub struct Compound<N: Real> {
    shapes: Vec<(Isometry<N>, ShapeHandle<N>)>,
    bvt: BVT<usize, AABB<N>>,
    bvs: Vec<AABB<N>>,
    nbits: usize
}

impl<N: Real> Compound<N> {
    /// Builds a new compound shape.
    pub fn new(shapes: Vec<(Isometry<N>, ShapeHandle<N>)>) -> Compound<N> {
        let mut bvs = Vec::new();
        let mut leaves = Vec::new();

        for (i, &(ref delta, ref shape)) in shapes.iter().enumerate() {
            // loosen for better persistancy
            let bv = shape.as_ref().aabb(delta).loosened(na::convert(0.04f64));

            bvs.push(bv.clone());
            leaves.push((i, bv));

            if let Some(comp) = shape.as_composite_shape() {
                panic!("Nested composite shapes are not allowed.");
            }
        }

        let nbits = mem::size_of::<usize>() * 8 - leaves.len().leading_zeros() as usize;
        let bvt = BVT::new_balanced(leaves);

        Compound {
            shapes: shapes,
            bvt: bvt,
            bvs: bvs,
            nbits,
        }
    }
}

impl<N: Real> Compound<N> {
    /// The shapes of this compound shape.
    #[inline]
    pub fn shapes(&self) -> &[(Isometry<N>, ShapeHandle<N>)] {
        &self.shapes[..]
    }

    /// The optimization structure used by this compound shape.
    #[inline]
    pub fn bvt(&self) -> &BVT<usize, AABB<N>> {
        &self.bvt
    }

    /// The shapes bounding volumes.
    #[inline]
    pub fn bounding_volumes(&self) -> &[AABB<N>] {
        &self.bvs[..]
    }

    /// The AABB of the i-th shape compositing this compound.
    #[inline]
    pub fn aabb_at(&self, i: usize) -> &AABB<N> {
        &self.bvs[i]
    }

    pub fn subshape_feature_id(&self, fid: FeatureId) -> (usize, FeatureId) {
        match fid {
            FeatureId::Face(i) => ((i & !(usize::max_value() << self.nbits)), FeatureId::Face(i >> self.nbits)),
            #[cfg(feature = "dim3")]
            FeatureId::Edge(i) => ((i & !(usize::max_value() << self.nbits)), FeatureId::Edge(i >> self.nbits)),
            FeatureId::Vertex(i) => ((i & !(usize::max_value() << self.nbits)), FeatureId::Vertex(i >> self.nbits)),
            FeatureId::Unknown => (0, FeatureId::Unknown)
        }
    }
}

impl<N: Real> CompositeShape<N> for Compound<N> {
    #[inline]
    fn nparts(&self) -> usize {
        self.shapes.len()
    }

    #[inline(always)]
    fn map_part_at(
        &self,
        i: usize,
        m: &Isometry<N>,
        f: &mut FnMut(&Isometry<N>, &Shape<N>),
    )
    {
        let elt = &self.shapes()[i];
        let pos = m * elt.0;

        f(&pos, elt.1.as_ref())
    }

    fn map_part_and_preprocessor_at(
        &self,
        i: usize,
        m: &Isometry<N>,
        prediction: &ContactPrediction<N>,
        f: &mut FnMut(&Isometry<N>, &Shape<N>, &ContactPreprocessor<N>),
    ) {
        let elt = &self.shapes()[i];
        let pos = m * elt.0;
        let proc = CompoundContactProcessor::new(&elt.0, i, self.nbits);

        f(&pos, elt.1.as_ref(), &proc)
    }

    #[inline]
    fn aabb_at(&self, i: usize) -> AABB<N> {
        self.bounding_volumes()[i].clone()
    }

    #[inline]
    fn bvh(&self) -> BVHImpl<N, usize, AABB<N>> {
        BVHImpl::BVT(&self.bvt)
    }
}


struct CompoundContactProcessor<'a, N: Real> {
    part_pos: &'a Isometry<N>,
    part_id: usize,
    nbits: usize
}

impl<'a, N: Real> CompoundContactProcessor<'a, N> {
    pub fn new(part_pos: &'a Isometry<N>, part_id: usize, nbits: usize) -> Self {
        CompoundContactProcessor {
            part_pos, part_id, nbits
        }
    }
}

impl<'a, N: Real> ContactPreprocessor<N> for CompoundContactProcessor<'a, N> {
    fn process_contact(
        &self,
        c: &mut Contact<N>,
        kinematic: &mut ContactKinematic<N>,
        is_first: bool)
        -> bool {
        // Fix the feature ID.
        let feature = if is_first {
            kinematic.feature1()
        } else {
            kinematic.feature2()
        };

        let actual_feature = match feature {
            FeatureId::Vertex(i) => FeatureId::Vertex((i << self.nbits) | self.part_id),
            #[cfg(feature = "dim3")]
            FeatureId::Edge(i) => FeatureId::Edge((i << self.nbits) | self.part_id),
            FeatureId::Face(i) => FeatureId::Face((i << self.nbits) | self.part_id),
            FeatureId::Unknown => return false,
        };

        if is_first {
            kinematic.set_feature1(actual_feature);
            kinematic.transform1(self.part_pos);
        } else {
            kinematic.set_feature2(actual_feature);
            kinematic.transform2(self.part_pos);
        }

        true
    }
}