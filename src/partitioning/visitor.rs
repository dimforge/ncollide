use partitioning::PartitioningStructure;

/// The status of the spatial partitoning structure traversal.
pub enum VisitStatus {
    /// The traversal should continue on the children of the currently visited nodes.
    Continue,
    /// The traversal should not be executed on the children of the currently visited nodes.
    Stop,
    /// The traversal should exit immediately.
    ExitEarly,
}

/// Trait implemented by visitor called during the traversal of a spatial partitioning data structure.
pub trait Visitor<T, BV> {
    /// Execute an operation on the content of a node of the spatial partitioning structure.
    ///
    /// Returns whether the traversal should continue on the node's children, if it should not continue
    /// on those children, or if the whole traversal should be exited early.
    fn visit(&mut self, bv: &BV, data: Option<&T>) -> VisitStatus;
}

/// Executes a traversal of a spatial partitioning data structure.
pub fn visit<P, T, BV>(part: &P,
                       visitor: &mut impl Visitor<T, BV>,
                       stack: &mut Vec<P::Node>)
    where P: PartitioningStructure<T, BV> {
    if let Some(root) = part.root() {
        stack.push(root);

        while let Some(node) = stack.pop() {
            let content = part.content(node);

            match visitor.visit(content.0, content.1) {
                VisitStatus::Continue => {
                    for i in 0..part.num_children(node) {
                        stack.push(part.child(i, node))
                    }
                }
                VisitStatus::ExitEarly => return,
                VisitStatus::Stop => {}
            }
        }
    }
}