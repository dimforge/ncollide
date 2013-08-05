use std::util;
use std::num::{Zero, One};
use std::iterator::IteratorUtil;
use std::vec;
use std::at_vec;
use extra::treemap::TreeMap;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::sub_dot::SubDot;
use nalgebra::traits::scalar_op::{ScalarMul, ScalarDiv};
use narrow::algorithm::simplex::Simplex;

///  Simplex using the Johnson subalgorithm to compute the projection of the origin on the simplex.
#[deriving(Eq, ToStr, Clone)] // FIXME: DeepClone
pub struct JohnsonSimplex<N, V> {
    priv recursion_template: @[RecursionTemplate<V>],
    priv points:             ~[V],
    priv exchange_points:    ~[V],
    priv cofactors:          ~[N]
}

/// Set of indices to explain to the JohnsonSimplex how to do its work.
/// Building this is very time consuming, and thus should be shared between all instances of the
/// Johnson simplex.
///
/// # Parameters:
///     * V - Parameter used to synchronize the template dimension and the simplex dimension.
#[deriving(Eq, Clone)] // FIXME: DeepClone
pub struct RecursionTemplate<V> {
    // FIXME: why #[doc(hidden)] does not work?
    /// For internal uses. Do not read.
    permutation_list: ~[uint],
    /// For internal uses. Do not read.
    offsets:          ~[uint],
    /// For internal uses. Do not read.
    sub_cofactors:    ~[uint],
    /// For internal uses. Do not read.
    num_cofactors:    uint,
    /// For internal uses. Do not read.
    num_leaves:       uint // useful only for printing…
}

impl<V: Dim> RecursionTemplate<V> {
    /// Creates a new set o Recursion simplex sharable between any Johnson simplex parametrized
    /// with the same `V`.
    pub fn new() -> @[RecursionTemplate<V>] {
        do at_vec::build |push| {
            for dim in range(0u, Dim::dim::<V>() + 1u) {
                push(RecursionTemplate::make_permutation_list::<V>(dim))
            }
        }
    }

    // This is the tricky part of the algorithm. This generates all datas needed
    // to run the johson subalgorithm fastly. This should _not_ be run every time
    // the algorithm is executed. Instead, it should be pre-computed, or computed
    // only once for all. The resulting GC-managed list is intented to be shared
    // between all other simplicis with the same dimension.
    fn make_permutation_list(dim: uint) -> RecursionTemplate<V> {
        // FIXME: the permutation list should be computed once for all, at compile
        // time. I dont know how to do that though…

        // The number of points on the biggest subsimplex
        let max_num_points      = dim + 1;

        let mut pts             = ~[]; // the result
        let mut offsets         = ~[];
        let mut sub_cofactors   = ~[];

        // the beginning of the last subsimplices list
        let mut last_dim_begin  = 0;

        // the end of the last subsimplices list
        let mut last_dim_end    = dim + 1 + 1;

        // the number of points of the last subsimplices
        let mut last_num_points = dim + 1;

        let mut map             = TreeMap::new::<~[uint], uint>();

        let mut cofactor_index  = 0;

        for i in range(0, max_num_points) {
            pts.push(i)
        }

        // initially push the whole simplex (will be removed at the end)
        pts.push(0);

        offsets.push(max_num_points + 1); 

        // ... then remove one point each time
        for i in range(0u, dim + 1) {
            // for each sub-simplex ...
            let mut curr      = last_dim_begin;
            let mut num_added = 0;

            while (curr != last_dim_end) {
                // ... iterate on it ...
                for j in range(0u, last_num_points) {
                    // ... and build all the sublist with one last point
                    let mut sublist = ~[];

                    // then extract the sub-simplex
                    for k in range(0u, last_num_points) {
                        // we remove the j'th point
                        if pts[curr + j] != pts[curr + k] {
                            sublist.push(pts[curr + k]);
                        }
                    }

                    // keep a trace of the removed point
                    sublist.push(pts[curr + j]);

                    match map.find(&sublist) {
                        Some(&v) => sub_cofactors.push(v),
                        None     => {
                            for &e in sublist.iter() {
                                pts.push(e);
                                num_added = num_added + 1;
                            }
                            sub_cofactors.push(cofactor_index);
                            map.insert(sublist, cofactor_index);
                            cofactor_index = cofactor_index + 1;
                        }
                    }
                }

                let mut parent = ~[];
                for k in range(0u, last_num_points + 1) {
                    parent.push(pts[curr + k])
                }


                match map.find(&parent) {
                    Some(&p) => sub_cofactors.push(p),
                    None => {
                        sub_cofactors.push(cofactor_index);
                        // There is no need to keep a place for the full simplex cofactor.
                        // So we dont increase the cofactor buffer index for the first
                        // iteration.
                        cofactor_index = cofactor_index + if i == 0 { 0 } else { 1 };
                    }
                }

                curr = curr + last_num_points + 1;
            }

            // initialize the next iteration with one less point
            last_dim_begin = last_dim_end ;
            last_dim_end = last_dim_end + num_added;
            offsets.push(last_dim_end);
            last_num_points = last_num_points - 1;
        }

        // cofactor indices for leaves
        for i in range(0u, max_num_points) {
            sub_cofactors.push(*map.find(&~[max_num_points - 1 - i]).unwrap())
        }

        // end to begin offsets
        offsets.unshift(0u);
        offsets.reverse();
        offsets.pop();

        let rev_offsets = offsets.map(|&e| pts.len() - e);
        let num_leaves = rev_offsets[0];

        // reverse points and cofectors
        pts.reverse();
        sub_cofactors.reverse();

        // remove the full simplex
        let num_pts = pts.len();
        pts.truncate(num_pts - max_num_points - 1);
        sub_cofactors.truncate(num_pts - max_num_points - 1);

        RecursionTemplate {
            offsets:          rev_offsets,
            permutation_list: pts,
            num_cofactors:    sub_cofactors[0] + 1,
            sub_cofactors:    sub_cofactors,
            num_leaves:       num_leaves
        }
    }
}

impl<N: Clone + Zero, V: Dim>
JohnsonSimplex<N, V> {
    /// Creates a new johnson simplex given a point.
    pub fn new(recursion: @[RecursionTemplate<V>], initial_point: V) -> JohnsonSimplex<N, V> {
        let _dim       = Dim::dim::<V>();
        let expoints   = vec::with_capacity(_dim);
        let mut points = vec::with_capacity(_dim);

        points.push(initial_point);

        JohnsonSimplex {
            points:               points
            , exchange_points:    expoints
            , cofactors:          vec::from_elem(recursion[_dim].num_cofactors, Zero::zero())
            , recursion_template: recursion
        }
    }
}

impl<N: Ord + Clone + Eq + DivisionRing + Ord + Bounded,
V: Clone + VectorSpace<N> + Norm<N> + SubDot<N> + Eq + Dim>
JohnsonSimplex<N, V> {
    fn do_project_origin(&mut self, reduce: bool) -> V {
        // FIXME: for optimization: do special case when there are only 1 point ?
        let _0                   = Zero::zero::<N>();
        let _1                   = One::one::<N>();
        let max_num_pts          = self.points.len();
        let recursion            = &self.recursion_template[max_num_pts - 1];
        let mut curr_num_pts     = 1u;
        let mut curr             = max_num_pts;

        for i in range(0u, max_num_pts) {
            self.cofactors[recursion.num_cofactors - 1 - i] = _1.clone();
        }

        /*
         * first loop: compute all the cofactors
         */
        let mut recursion_offsets_skip_2 = recursion.offsets.iter();
        recursion_offsets_skip_2.next(); // Skip the two first entries
        recursion_offsets_skip_2.next();
        for &end in recursion_offsets_skip_2 {
            // for each sub-simplex ...
            let mut _i = curr;
            while (_i != end) {
                let mut cofactor = Zero::zero::<N>();
                let j_pid        = recursion.permutation_list[curr];
                let k_pid        = recursion.permutation_list[curr + 1u];

                // ... with curr_num_pts points ...
                for i in range(0u, curr_num_pts) {
                    // ... compute its cofactor.
                    let i_pid        = recursion.permutation_list[curr + 1 + i];
                    let sub_cofactor = self.cofactors[recursion.sub_cofactors[curr + 1 + i]].clone();
                    let delta = sub_cofactor *
                        self.points[k_pid].sub_dot(&self.points[j_pid],
                        &self.points[i_pid]);

                    cofactor = cofactor + delta;
                }

                self.cofactors[recursion.sub_cofactors[curr]] = cofactor;
                curr = curr + curr_num_pts + 1; // points + removed point + cofactor id

                _i = _i + curr_num_pts + 1;
            }

            curr_num_pts = curr_num_pts + 1;
        }

        /*
         * second loop: find the subsimplex containing the projection
         */
        let _invalid_cofactor = Bounded::max_value::<N>();
        let mut offsets_iter = recursion.offsets.rev_iter();
        offsets_iter.next(); // skip the first offset
        for &end in offsets_iter {
            // for each sub-simplex ...
            let mut _i = curr;
            while (_i != end) {
                let mut foundit = true;
                // ... with curr_num_pts points permutations ...
                for i in range(0u, curr_num_pts) {
                    // ... see if its cofactor is positive
                    let cof_id = _i - (i + 1) * curr_num_pts;
                    let cof    = self.cofactors[recursion.sub_cofactors[cof_id]].clone();
                    if cof > _0 {
                        // invalidate the children cofactor
                        if curr_num_pts > 1 {
                            let subcofid = recursion.sub_cofactors[cof_id + 1];

                            if self.cofactors[subcofid] > _0 {
                                self.cofactors[subcofid] = _invalid_cofactor.clone()
                            }
                        }

                        // dont concider this sub-simplex if it has been invalidated by its
                        // parent(s)
                        if cof == _invalid_cofactor {
                            foundit = false
                        }
                    }
                    else {
                        // we found a negative cofactor: no projection possible here
                        foundit = false
                    }
                }

                if foundit {
                    // we found a projection!
                    // re-run the same iteration but, this time, compute the projection
                    let mut total_cof = _0.clone();
                    let mut proj      = Zero::zero::<V>();

                    for i in range(0u, curr_num_pts) {
                        // ... see if its cofactor is positive
                        let id    = _i - (i + 1) * curr_num_pts;
                        let cof   = self.cofactors[recursion.sub_cofactors[id]].clone();

                        total_cof = total_cof + cof;
                        proj = proj +
                            self.points[recursion.permutation_list[id]].scalar_mul(&cof);
                    }

                    if reduce {
                        // we need to reduce the simplex 
                        // FIXME: is it possible to do that without copy?
                        // Maybe with some kind of cross-vector-swap?
                        for i in range(0u, curr_num_pts) {
                            let id = _i - (i + 1) * curr_num_pts;
                            self.exchange_points.push(self.points[recursion.permutation_list[id]].clone());
                        }

                        util::swap(&mut self.exchange_points, &mut self.points);
                        self.exchange_points.clear();
                    }

                    proj.scalar_div_inplace(&total_cof);

                    return proj
                }

                _i = _i - curr_num_pts * curr_num_pts;
            }

            curr = end;
            curr_num_pts = curr_num_pts - 1;
        }

        // println(self.points.to_str());
        // println(self.cofactors.to_str());
        Zero::zero()
            // fail!("Internal error: projection of the origin failed.");
    }
}

impl<V: Clone + VectorSpace<N> + Norm<N> + SubDot<N> + Eq + Dim,
N: Ord + Clone + Eq + DivisionRing + Ord + Bounded>
Simplex<N, V> for JohnsonSimplex<N, V> {
    fn reset(&mut self, pt: V) {
        self.points.clear();
        self.points.push(pt);
    }

    fn dimension(&self) -> uint {
        self.points.len() - 1
    }

    fn max_sq_len(&self) -> N {
        self.points.iter().transform(|v| v.sqnorm()).max().unwrap()
    }

    fn contains_point(&self, pt: &V) -> bool {
        self.points.iter().any(|v| pt == v)
    }

    fn add_point(&mut self, pt: V) {
        self.points.push(pt);
        assert!(self.points.len() <= Dim::dim::<V>() + 1);
    }

    fn project_origin_and_reduce(&mut self) -> V {
        self.do_project_origin(true)
    }

    fn project_origin(&mut self) -> V {
        self.do_project_origin(false)
    }
}

impl<V> ToStr for RecursionTemplate<V> {
    fn to_str(&self) -> ~str {
        let mut res  = ~"RecursionTemplate { ";
        let mut curr = self.num_leaves;
        let mut dim  = 1;

        res = res + "num_cofactors: " + self.num_cofactors.to_str();

        let mut recursion_offsets_skip_1 = self.offsets.iter();
        recursion_offsets_skip_1.next(); // Skip the two first entries

        for &off in recursion_offsets_skip_1 {
            while (curr != off) {
                res = res + "\n(@" + self.sub_cofactors[curr].to_str() + " -> ";

                for i in range(0u, dim) {
                    res = res + self.permutation_list[i + curr].to_str();
                    if i != dim - 1 {
                        res = res + " ";
                    }
                }

                res = res + " - ";

                for i in range(1u, dim) {
                    res = res + self.sub_cofactors[i + curr].to_str();
                    if i != dim - 1 {
                        res = res + " ";
                    }
                }

                res  = res + ")";
                curr = curr + dim;
            }

            dim = dim + 1;
        }

        res = res + " }\n";

        res = res + "offsets: " + self.offsets.to_str();

        res
    }
}
