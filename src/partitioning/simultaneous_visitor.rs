use partitioning::{BVH, VisitStatus};

/// Trait implemented by visitor called during a simultaneous spatial partitioning data structure tarversal.
pub trait SimultaneousVisitor<T, BV> {
    /// Execute an operation on the content of two nodes, one from each structure.
    ///
    /// Returns whether the traversal should continue on the nodes children, if it should not continue
    /// on those children, or if the whole traversal should be exited early.
    fn visit(&mut self, left_bv: &BV, left_data: Option<&T>, right_bv: &BV, right_data: Option<&T>) -> VisitStatus;
}


/// Executes a simultaneous traversal of two spatial partitioning data structures.
pub fn simultaneous_visit<H1, H2, T, BV>(hierarchy1: &H1,
                                         hierarchy2: &H2,
                                         visitor: &mut impl SimultaneousVisitor<T, BV>,
                                         stack: &mut Vec<(H1::Node, H2::Node)>)
    where H1: BVH<T, BV>,
          H2: BVH<T, BV> {
    if let (Some(root1), Some(root2)) = (hierarchy1.root(), hierarchy2.root()) {
        stack.push((root1, root2));

        while let Some((node1, node2)) = stack.pop() {
            let content1 = hierarchy1.content(node1);
            let content2 = hierarchy2.content(node2);

            match visitor.visit(content1.0, content1.1, content2.0, content2.1) {
                VisitStatus::Continue => {
                    for
                        i in 0..hierarchy1.num_children(node1) {
                        let n1 = hierarchy1.child(i, node1);

                        for j in 0..hierarchy2.num_children(node2) {
                            let n2 = hierarchy2.child(j, node2);
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