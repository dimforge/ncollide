/// Trait implemented by Bounding Volume Hierarchy.
pub trait BVH<T, BV> {
    /// Type of a node of this BVH.
    type Node: Copy;

    /// The root of the BVH.
    fn root(&self) -> Option<Self::Node>;
    /// The number of children of the given node.
    fn num_children(&self, node: Self::Node) -> usize;
    /// The i-th child of the given node.
    fn child(&self, i: usize, node: Self::Node) -> Self::Node;
    /// The bounding volume and data contained by the given node.
    fn content(&self, node: Self::Node) -> (&BV, Option<&T>);
}