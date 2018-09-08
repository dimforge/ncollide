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


/// Trait implemented by visitor called during a simultaneous spatial partitioning data structure tarversal.
pub trait SimultaneousVisitor<T, BV> {
    /// Execute an operation on the content of two nodes, one from each structure.
    ///
    /// Returns whether the traversal should continue on the nodes children, if it should not continue
    /// on those children, or if the whole traversal should be exited early.
    fn visit(&mut self, left_bv: &BV, left_data: Option<&T>, right_bv: &BV, right_data: Option<&T>) -> VisitStatus;
}


/// The next action to be taken by a AbstractBVH traversal algorithm after having visited a node with a bounding volume.
pub enum BestFirstBVVisitStatus<N> {
    /// The traversal continues recursively, associating the given cost to the visited node.
    ContinueWithCost(N),
    // FIXME: rename this to StopPropagation?
    /// The traversal does not continue recursively on the descendants of this node (but continues on other nodes).
    Stop,
    /// The traversal aborts, returning the last best result found.
    ExitEarly,
}

/// The next action to be taken by a AbstractBVH traversal algorithm after having visited a node with some data.
pub enum BestFirstDataVisitStatus<N, Res> {
    /// The traversal continues recursively on the descendants of this node, if any. The given result associated by a cost value are registered.
    ContinueWithResult(N, Res),
    /// The traversal continues recursively on the descendant of this node.
    Continue,
    /// The traversal aborts, returning the given result.
    ExitEarlyWithResult(Res),
    /// The traversal aborts, returnin the last best result found.
    ExitEarly,
}

/// Trait implemented by cost functions used by the best-first search on a `BVT`.
pub trait BestFirstVisitor<N, T, BV> {
    /// The result of a best-fist traversal.
    type Result;

    /// Compute the next action to be taken by the best-first-search after visiting a node containing the given bounding volume.
    fn visit_bv(&mut self, bv: &BV) -> BestFirstBVVisitStatus<N>;
    /// Compute the next action to be taken by the best-first-search after visiting a node containing the given data.
    fn visit_data(&mut self, data: &T) -> BestFirstDataVisitStatus<N, Self::Result>;
}
