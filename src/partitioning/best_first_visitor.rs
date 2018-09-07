use na::Real;
use partitioning::BVH;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// The next action to be taken by a BVH traversal algorithm after having visited a node with a bounding volume.
pub enum BestFirstBVVisitStatus<N> {
    /// The traversal continues recursively, associating the given cost to the visited node.
    ContinueWithCost(N),
    // FIXME: rename this to StopPropagation?
    /// The traversal does not continue recursively on the descendants of this node (but continues on other nodes).
    Stop,
    /// The traversal aborts, returning the last best result found.
    ExitEarly,
}

/// The next action to be taken by a BVH traversal algorithm after having visited a node with some data.
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

/// Performs a best-first traversal on the given BVH, i.e., a traversal that finds the result with the smalles tassociated cost.
pub fn best_first_visit<H, N, T, BV, Vis>(hierarchy: &H, visitor: &mut Vis) -> Option<Vis::Result>
    where
        N: Real,
        H: BVH<T, BV>,
        Vis: BestFirstVisitor<N, T, BV> {
    let mut queue: BinaryHeap<WeightedValue<N, H::Node>> = BinaryHeap::new();
    let mut best_cost = N::max_value();
    let mut result = None;

    if let Some(root) = hierarchy.root() {
        let root_content = hierarchy.content(root);

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

            for i in 0..hierarchy.num_children(entry.value) {
                let child = hierarchy.child(i, entry.value);
                let content = hierarchy.content(child);

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
