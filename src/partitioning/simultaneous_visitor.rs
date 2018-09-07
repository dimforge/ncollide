use partitioning::{PartitioningStructure, VisitStatus};

/// Trait implemented by visitor called during a simultaneous spatial partitioning data structure tarversal.
pub trait SimultaneousVisitor<T, BV> {
    /// Execute an operation on the content of two nodes, one from each structure.
    ///
    /// Returns whether the traversal should continue on the nodes children, if it should not continue
    /// on those children, or if the whole traversal should be exited early.
    fn visit_pair(&mut self, left_bv: &BV, left_data: Option<&T>, right_bv: &BV, right_data: Option<&T>) -> VisitStatus;
}


/// Executes a simultaneous traversal of two spatial partitioning data structures.
pub fn simultaneous_visit<P1, P2, T, BV>(part1: &P1,
                                         part2: &P2,
                                         visitor: &mut impl SimultaneousVisitor<T, BV>,
                                         stack: &mut Vec<(P1::Node, P2::Node)>)
    where P1: PartitioningStructure<T, BV>,
          P2: PartitioningStructure<T, BV> {
    if let (Some(root1), Some(root2)) = (part1.root(), part2.root()) {
        stack.push((root1, root2));

        while let Some((node1, node2)) = stack.pop() {
            let content1 = part1.content(node1);
            let content2 = part2.content(node2);

            match visitor.visit_pair(content1.0, content1.1, content2.0, content2.1) {
                VisitStatus::Continue => {
                    for
                        i in 0..part1.num_children(node1) {
                        let n1 = part1.child(i, node1);

                        for j in 0..part2.num_children(node2) {
                            let n2 = part2.child(i, node2);
                            stack.push((n1, n2))
                        }
                    }
                }
                VisitStatus::ExitEarly => return,
                VisitStatus::Stop => {}
            }
        }
    }
}