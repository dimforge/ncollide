pub trait PartitioningStructure<T, BV> {
    type Node: Copy;

    fn root(&self) -> Option<Self::Node>;
    fn num_children(&self, node: Self::Node) -> usize;
    fn child(&self, i: usize, node: Self::Node) -> Self::Node;
    fn content(&self, node: Self::Node) -> (&BV, Option<&T>);
}