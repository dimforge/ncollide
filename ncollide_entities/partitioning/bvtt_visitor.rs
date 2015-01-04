/// Visitor for the Bounding Volume Traversal Tree.
// FIXME: make this more generic using <B1, BV1, B2, BV2> ?
pub trait BVTTVisitor<B, BV> {
    /// Visit two internal nodes.
    fn visit_internal_internal(&mut self, &BV, &BV) -> bool;
    /// Visit two leaves.
    fn visit_leaf_leaf(&mut self, &B, &BV, &B, &BV);
    /// Visit one internal node and one leaf.
    fn visit_internal_leaf(&mut self, &BV, &B, &BV) -> bool;
    /// Visit one leaf and on internal node.
    fn visit_leaf_internal(&mut self, &B, &BV, &BV) -> bool;
}
