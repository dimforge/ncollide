//!
//! Shape composed from the union of primitives.
//!

use std::mem;
use crate::bounding_volume::{BoundingVolume, AABB};
use crate::math::Isometry;
use na::{self, RealField};
use crate::partitioning::{BVHImpl, BVT};
use crate::shape::{CompositeShape, Shape, ShapeHandle, FeatureId};
use crate::query::{ContactPrediction, ContactPreprocessor, Contact, ContactKinematic};

/// A compound shape with an aabb bounding volume.
///
/// A compound shape is a shape composed of the union of several simpler shape. This is
/// the main way of creating a concave shape from convex parts. Each parts can have its own
/// delta transformation to shift or rotate it with regard to the other shapes.
#[derive(Clone)]
pub struct Compound<N: RealField> {
    shapes: Vec<(Isometry<N>, ShapeHandle<N>)>,
    bvt: BVT<usize, AABB<N>>,
    bvs: Vec<AABB<N>>,
    nbits: usize
}

impl<N: RealField> Compound<N> {
    /// Builds a new compound shape.
    pub fn new(shapes: Vec<(Isometry<N>, ShapeHandle<N>)>) -> Compound<N> {
        let mut bvs = Vec::new();
        let mut leaves = Vec::new();

        for (i, &(ref delta, ref shape)) in shapes.iter().enumerate() {
            // loosen for better persistancy
            let bv = shape.as_ref().aabb(delta).loosened(na::convert(0.04f64));

            bvs.push(bv.clone());
            leaves.push((i, bv));

            if let Some(_comp) = shape.as_composite_shape() {
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

impl<N: RealField> Compound<N> {
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

    /// The AABB of this compound in its local-space.
    #[inline]
    pub fn aabb(&self) -> &AABB<N> {
        self.bvt().root_bounding_volume()
            .expect("An empty Compound has no AABB.")
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

    /// Transforms a FeatureId of this compound into a pair containing the index of the subshape
    /// containing this feature, and the corresponding FeatureId on this subshape.
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

impl<N: RealField> CompositeShape<N> for Compound<N> {
    #[inline]
    fn nparts(&self) -> usize {
        self.shapes.len()
    }

    #[inline(always)]
    fn map_part_at(
        &self,
        i: usize,
        m: &Isometry<N>,
        f: &mut dyn FnMut(&Isometry<N>, &dyn Shape<N>),
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
        _prediction: &ContactPrediction<N>,
        f: &mut dyn FnMut(&Isometry<N>, &dyn Shape<N>, &dyn ContactPreprocessor<N>),
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


struct CompoundContactProcessor<'a, N: RealField> {
    part_pos: &'a Isometry<N>,
    part_id: usize,
    nbits: usize
}

impl<'a, N: RealField> CompoundContactProcessor<'a, N> {
    pub fn new(part_pos: &'a Isometry<N>, part_id: usize, nbits: usize) -> Self {
        CompoundContactProcessor {
            part_pos, part_id, nbits
        }
    }
}

impl<'a, N: RealField> ContactPreprocessor<N> for CompoundContactProcessor<'a, N> {
    fn process_contact(
        &self,
        _c: &mut Contact<N>,
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
            // The contact kinematics must be expressed on the local frame of
            // the compound instead of the sub-shape.
            kinematic.transform1(self.part_pos);
        } else {
            kinematic.set_feature2(actual_feature);
            // The contact kinematics must be expressed on the local frame of
            // the compound instead of the sub-shape.
            kinematic.transform2(self.part_pos);
        }

        true
    }
}