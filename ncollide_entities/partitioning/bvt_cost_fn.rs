/// Trait implemented by cost functions used by the best-first search on a `BVT`.
pub trait BVTCostFn<N, B, BV, R> {
    /// Computes the cost of a bounding volume.
    fn compute_bv_cost(&mut self, &BV) -> Option<N>;
    /// Computes the cost of an object, and the result to be returned if it is the best one.
    fn compute_b_cost(&mut self, &B) -> Option<(N, R)>;
}
