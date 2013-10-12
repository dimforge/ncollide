use nalgebra::na::{AlgebraicVecExt, Cast, VecExt, Translation, AbsoluteRotate, Transform};
use bounding_volume::{LooseBoundingVolume, AABB, HasAABB};
use partitioning::bvt::BVT;
use partitioning::bvt;

/// A compound geometry with an aabb bounding volume. A compound geometry is a geometry composed of
/// the union of several simpler geometry. This is the main way of creating a concave geometry from
/// convex parts. Each parts can have its own delta transformation to shift or rotate it with
/// regard to the other geometries.
#[deriving(Encodable, Decodable)]
pub struct CompoundAABB<N, V, M, S> {
    priv shapes: ~[(M, S)],
    priv bvt:    BVT<uint, AABB<N, V>>,
    priv bvs:    ~[AABB<N, V>]
}

impl<N: 'static + Algebraic + Primitive + Orderable + Signed + Cast<f32> + Clone,
     V: 'static + AlgebraicVecExt<N> + Clone,
     M,
     S: HasAABB<N, V, M>>
CompoundAABB<N, V, M, S> {
    /// Builds a new compound shape from a list of shape with their respective delta
    /// transformation.
    pub fn new(shapes: ~[(M, S)]) -> CompoundAABB<N, V, M, S> {
        let mut bvs    = ~[];
        let mut leaves = ~[];

        for (i, &(ref delta, ref shape)) in shapes.iter().enumerate() {
            let bv = shape.aabb(delta).loosened(Cast::from(0.04)); // loosen for better persistancy

            bvs.push(bv.clone());
            leaves.push((i, bv));
        }

        let bvt = BVT::new_with_partitioner(leaves, bvt::dim_pow_2_aabb_partitioner);

        CompoundAABB {
            shapes: shapes,
            bvt:    bvt,
            bvs:    bvs
        }
    }
}

impl<N, V, M, S> CompoundAABB<N, V, M, S> {
    /// The shapes of this compound geometry.
    #[inline]
    pub fn shapes<'r>(&'r self) -> &'r [(M, S)] {
        let res: &'r [(M, S)] = self.shapes;

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

impl<N: Primitive + Orderable + Cast<f32>,
     V: VecExt<N> + Clone,
     M: Mul<M, M> + Translation<V> + AbsoluteRotate<V> + Transform<V>,
     S: HasAABB<N, V, M>>
HasAABB<N, V, M> for CompoundAABB<N, V, M, S> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<N, V> {
        let bv              = self.bvt.root_bounding_volume().unwrap();
        let ls_center       = bv.translation();
        let center          = m.transform(&ls_center);
        let half_extents    = (bv.maxs() - *bv.mins()) / Cast::from(2.0);
        let ws_half_extents = m.absolute_rotate(&half_extents);

        AABB::new(center - ws_half_extents, center + ws_half_extents)
    }
}
