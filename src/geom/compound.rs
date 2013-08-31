use nalgebra::vec::{AlgebraicVecExt, VecExt};
use bounding_volume::{BoundingVolume, LooseBoundingVolume, AABB, HasAABB};
use partitioning::dbvt::{DBVT, DBVTLeaf};

/// A compound geometry with an aabb bounding volume. A compound geometry is a geometry composed of
/// the union of several simpler geometry. This is the main way of creating a concave geometry from
/// convex parts. Each parts can have its own delta transformation to shift or rotate it with
/// regard to the other geometries.
pub struct CompoundAABB<N, V, M, S> {
    priv shapes: ~[(M, S)],
    priv dbvt:   DBVT<V, uint, AABB<N, V>>,
    priv leaves: ~[@mut DBVTLeaf<V, uint, AABB<N, V>>]
}

impl<N: 'static + Algebraic + Primitive + Orderable + ToStr,
     V: 'static + AlgebraicVecExt<N> + Clone + ToStr,
     M,
     S: HasAABB<N, V, M>>
CompoundAABB<N, V, M, S> {
    /// Builds a new compound shape from a list of shape with their respective delta
    /// transformation.
    pub fn new(shapes: ~[(M, S)]) -> CompoundAABB<N, V, M, S> {
        let mut dbvt   = DBVT::new();
        let mut leaves = ~[];

        // FIXME: shuffle the shapes array to avoid the dbvt worst case?

        for (i, &(ref delta, ref shape)) in shapes.iter().enumerate() {
            let bv = shape.aabb(delta).loosened(NumCast::from(0.04)); // loozen for better persistancy
            let l  = @mut DBVTLeaf::new(bv, i);
            leaves.push(l);
            dbvt.insert(l)
        }

        CompoundAABB {
            shapes: shapes,
            dbvt:   dbvt,
            leaves: leaves
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
    pub fn dbvt<'r>(&'r self) -> &'r DBVT<V, uint, AABB<N, V>> {
        &'r self.dbvt
    }

    /// The leaves of the bouding volume tree used by thes compound geometry.
    #[inline]
    pub fn leaves<'r>(&'r self) -> &'r [@mut DBVTLeaf<V, uint, AABB<N, V>>] {
        let res: &'r [@mut DBVTLeaf<V, uint, AABB<N, V>>] = self.leaves;

        res
    }
}

impl<N: Primitive + Orderable + ToStr,
     V: VecExt<N> + Clone + ToStr,
     M: Mul<M, M>,
     S: HasAABB<N, V, M>>
HasAABB<N, V, M> for CompoundAABB<N, V, M, S> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<N, V> {
        let mut iter          = self.shapes.iter();
        let &(ref m1, ref s1) = iter.next().expect("Cannot compute the aabb of an empty compoundAABB.");

        let mut res = s1.aabb(&(m * *m1));

        for &(ref mi, ref si) in iter {
            res.merge(&si.aabb(&(m * *mi)))
        }

        res
    }
}
