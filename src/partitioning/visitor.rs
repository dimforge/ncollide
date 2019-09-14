/// The status of the spatial partitioning structure traversal.
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

/// Trait implemented by visitor called during a simultaneous spatial partitioning data structure tarversal.
pub trait SimultaneousVisitor<T, BV> {
    /// Execute an operation on the content of two nodes, one from each structure.
    ///
    /// Returns whether the traversal should continue on the nodes children, if it should not continue
    /// on those children, or if the whole traversal should be exited early.
    fn visit(
        &mut self,
        left_bv: &BV,
        left_data: Option<&T>,
        right_bv: &BV,
        right_data: Option<&T>,
    ) -> VisitStatus;
}

/// The next action to be taken by a BVH traversal algorithm after having visited a node with some data.
pub enum BestFirstVisitStatus<N, Res> {
    /// The traversal continues recursively, associating the given cost to the visited node and some associated result.
    Continue {
        /// The cost associated to this node.
        cost: N,
        /// The result, if any, associated to this cost.
        result: Option<Res>,
    },
    /// The traversal does not continue recursively on the visited node's children.
    Stop,
    /// The traversal aborts.
    ///
    /// If a data is provided, then it is returned as the result of the traversal.
    /// If no result is provided, then the last best result found becomes the result of the traversal.
    ExitEarly(Option<Res>),
}

/// Trait implemented by cost functions used by the best-first search on a `BVT`.
pub trait BestFirstVisitor<N, T, BV> {
    /// The result of a best-first traversal.
    type Result;

    /// Compute the next action to be taken by the best-first-search after visiting a node containing the given bounding volume.
    fn visit(
        &mut self,
        best_cost_so_far: N,
        bv: &BV,
        value: Option<&T>,
    ) -> BestFirstVisitStatus<N, Self::Result>;
}
