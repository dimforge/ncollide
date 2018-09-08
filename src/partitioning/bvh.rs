use na::Real;
use partitioning::{BestFirstBVVisitStatus, BestFirstDataVisitStatus, BestFirstVisitor, BVT, DBVT, SimultaneousVisitor,
                   Visitor, VisitStatus};
use std::cmp::Ordering;
use std::collections::BinaryHeap;


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


    /// Traverses this BVH using a visitor.
    fn visit(&self, visitor: &mut impl Visitor<T, BV>) {
        // FIXME: find a way to avoid the allocation.
        let mut stack = Vec::new();

        if let Some(root) = self.root() {
            stack.push(root);

            while let Some(node) = stack.pop() {
                let content = self.content(node);

                match visitor.visit(content.0, content.1) {
                    VisitStatus::Continue => {
                        for i in 0..self.num_children(node) {
                            stack.push(self.child(i, node))
                        }
                    }
                    VisitStatus::ExitEarly => return,
                    VisitStatus::Stop => {}
                }
            }
        }
    }

    /// Visits the bounding volume test tree implicitly formed with `other`.
    fn visit_bvtt(&self, other: &impl BVH<T, BV>, visitor: &mut impl SimultaneousVisitor<T, BV>) {
        // FIXME: find a way to avoid the allocation.
        let mut stack = Vec::new();

        if let (Some(root1), Some(root2)) = (self.root(), other.root()) {
            stack.push((root1, root2));

            while let Some((node1, node2)) = stack.pop() {
                let content1 = self.content(node1);
                let content2 = other.content(node2);

                match visitor.visit(content1.0, content1.1, content2.0, content2.1) {
                    VisitStatus::Continue => {
                        for
                            i in 0..self.num_children(node1) {
                            let n1 = self.child(i, node1);

                            for j in 0..other.num_children(node2) {
                                let n2 = other.child(j, node2);
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

    /// Performs a best-fist-search on the BVH.
    ///
    /// Returns the content of the leaf with the smallest associated cost, and a result of
    /// user-defined type.
    fn best_first_search<N, BFS>(&self, visitor: &mut BFS) -> Option<BFS::Result>
        where
            N: Real,
            BFS: BestFirstVisitor<N, T, BV>,
    {
        let mut queue: BinaryHeap<WeightedValue<N, Self::Node>> = BinaryHeap::new();
        let mut best_cost = N::max_value();
        let mut result = None;

        if let Some(root) = self.root() {
            let root_content = self.content(root);

            match visitor.visit_bv(root_content.0) {
                BestFirstBVVisitStatus::ContinueWithCost(cost) => {
                    if let Some(data) = root_content.1 {
                        match visitor.visit_data(data) {
                            BestFirstDataVisitStatus::ContinueWithResult(res_cost, res) => {
                                best_cost = res_cost;
                                result = Some(res);
                            }
                            BestFirstDataVisitStatus::ExitEarlyWithResult(res) => return Some(res),
                            BestFirstDataVisitStatus::Continue => {}
                            BestFirstDataVisitStatus::ExitEarly => return None
                        }
                    }

                    queue.push(WeightedValue::new(root, -cost))
                }
                BestFirstBVVisitStatus::Stop | BestFirstBVVisitStatus::ExitEarly => return None
            }

            while let Some(entry) = queue.pop() {
                if -entry.cost >= best_cost {
                    break; // Solution found.
                }

                for i in 0..self.num_children(entry.value) {
                    let child = self.child(i, entry.value);
                    let content = self.content(child);

                    match visitor.visit_bv(content.0) {
                        BestFirstBVVisitStatus::ContinueWithCost(cost) => {
                            if cost < best_cost {
                                if let Some(data) = content.1 {
                                    match visitor.visit_data(data) {
                                        BestFirstDataVisitStatus::ContinueWithResult(res_cost, res) => {
                                            if res_cost < best_cost {
                                                best_cost = res_cost;
                                                result = Some(res)
                                            }
                                        }
                                        BestFirstDataVisitStatus::Continue => {}
                                        BestFirstDataVisitStatus::ExitEarly => return result,
                                        BestFirstDataVisitStatus::ExitEarlyWithResult(res) => return Some(res),
                                    }
                                }

                                queue.push(WeightedValue::new(child, -cost))
                            }
                        }
                        BestFirstBVVisitStatus::ExitEarly => return result,
                        BestFirstBVVisitStatus::Stop => {}
                    }
                }
            }
        }

        result
    }
}

/// An enum grouping references to all the BVH implementations on ncollide.
#[derive(Copy, Clone)]
pub enum BVHImpl<'a, N: 'a + Real, T: 'a, BV: 'a> {
    /// A static binary bounding volume tree.
    BVT(&'a BVT<T, BV>),
    /// A dynamic binary bounding volume tree.
    DBVT(&'a DBVT<N, T, BV>),
}

impl<'a, N: Real, T, BV> BVHImpl<'a, N, T, BV> {
    /// Traverses this tree using a visitor.
    pub fn visit(self, visitor: &mut impl Visitor<T, BV>) {
        match self {
            BVHImpl::BVT(bvt) => bvt.visit(visitor),
            BVHImpl::DBVT(dbvt) => dbvt.visit(visitor)
        }
    }

    /// Visits the bounding volume traversal tree implicitly formed with `other`.
    pub fn visit_bvtt<'b>(self, other: BVHImpl<'b, N, T, BV>, visitor: &mut impl SimultaneousVisitor<T, BV>) {
        // Note: the dispatch on each pair is split into two method to avoid
        // having to write a manually a match over each possible pair.
        match other {
            BVHImpl::BVT(bvh2) => self.visit_bvtt_dispatch(bvh2, visitor),
            BVHImpl::DBVT(bvh2) => self.visit_bvtt_dispatch(bvh2, visitor),
        }
    }

    fn visit_bvtt_dispatch(self, bvh2: &impl BVH<T, BV>, visitor: &mut impl SimultaneousVisitor<T, BV>) {
        match self {
            BVHImpl::BVT(bvh1) => bvh1.visit_bvtt(bvh2, visitor),
            BVHImpl::DBVT(bvh1) => bvh1.visit_bvtt(bvh2, visitor),
        }
    }

    /// Performs a best-fist-search on the tree.
    ///
    /// Returns the content of the leaf with the smallest associated cost, and a result of
    /// user-defined type.
    pub fn best_first_search<BFS>(self, visitor: &mut BFS) -> Option<BFS::Result>
        where
            BFS: BestFirstVisitor<N, T, BV>,
    {
        match self {
            BVHImpl::BVT(bvt) => bvt.best_first_search(visitor),
            BVHImpl::DBVT(dbvt) => dbvt.best_first_search(visitor)
        }
    }
}


struct WeightedValue<N, T> {
    pub value: T,
    pub cost: N,
}

impl<N, T> WeightedValue<N, T> {
    /// Creates a new reference packed with a cost value.
    #[inline]
    pub fn new(value: T, cost: N) -> WeightedValue<N, T> {
        WeightedValue {
            value: value,
            cost: cost,
        }
    }
}

impl<N: PartialEq, T> PartialEq for WeightedValue<N, T> {
    #[inline]
    fn eq(&self, other: &WeightedValue<N, T>) -> bool {
        self.cost.eq(&other.cost)
    }
}

impl<N: PartialEq, T> Eq for WeightedValue<N, T> {}

impl<N: PartialOrd, T> PartialOrd for WeightedValue<N, T> {
    #[inline]
    fn partial_cmp(&self, other: &WeightedValue<N, T>) -> Option<Ordering> {
        self.cost.partial_cmp(&other.cost)
    }
}

impl<N: PartialOrd, T> Ord for WeightedValue<N, T> {
    #[inline]
    fn cmp(&self, other: &WeightedValue<N, T>) -> Ordering {
        if self.cost < other.cost {
            Ordering::Less
        } else if self.cost > other.cost {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    }
}
