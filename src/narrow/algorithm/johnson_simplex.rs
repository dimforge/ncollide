//!  Simplex using the Johnson subalgorithm to compute the projection of the origin on the simplex.

use std::util;
use std::num::{Zero, One};
use std::vec;
use std::local_data;
use extra::arc::Arc;
use extra::treemap::TreeMap;
use nalgebra::na::{AlgebraicVec, Dim};
use nalgebra::na;
use narrow::algorithm::simplex::Simplex;

static KEY_RECURSION_TEMPLATE: local_data::Key<Arc<~[RecursionTemplate]>> = &local_data::Key;

///  Simplex using the Johnson subalgorithm to compute the projection of the origin on the simplex.
#[deriving(Clone)]
pub struct JohnsonSimplex<N, V> {
    priv recursion_template: Arc<~[RecursionTemplate]>,
    priv points:             ~[V],
    priv exchange_points:    ~[V],
    priv determinants:       ~[N]
}

/// Set of indices to explain to the JohnsonSimplex how to do its work.
/// Building this is very time consuming, and thus should be shared between all instances of the
/// Johnson simplex.
#[deriving(Eq, Clone, Encodable, Decodable)]
pub struct RecursionTemplate {
    // FIXME: why #[doc(hidden)] does not work?
    /// For internal uses. Do not read.
    permutation_list: ~[uint],
    /// For internal uses. Do not read.
    offsets:          ~[uint],
    /// For internal uses. Do not read.
    sub_determinants: ~[uint],
    /// For internal uses. Do not read.
    num_determinants: uint,
    /// For internal uses. Do not read.
    num_leaves:       uint // useful only for printing…
}

impl RecursionTemplate {
    /// Creates a new set o Recursion simplex sharable between any Johnson simplex having a
    /// dimension inferior or equal to `dim`.
    pub fn new(dim: uint) -> Arc<~[RecursionTemplate]> {
        let mut template = vec::with_capacity(dim + 1);

        for dim in range(0u, dim + 1u) {
            template.push(RecursionTemplate::make_permutation_list(dim))
        }

        Arc::new(template)
    }

    // pub fn to_raw_str(&self) -> ~str {
    //     let res = "permutation_list: " + self.permutation_list.to_str() + ", " +
    //               "offset: "           + self.offsets.to_str() + ", " +
    //               "sub_determinants: " + self.sub_determinants.to_str();

    //     res
    // }

    // This is the tricky part of the algorithm. This generates all datas needed
    // to run the johson subalgorithm fastly. This should _not_ be run every time
    // the algorithm is executed. Instead, it should be pre-computed, or computed
    // only once for all. The resulting GC-managed list is intented to be shared
    // between all other simplicis with the same dimension.
    fn make_permutation_list(dim: uint) -> RecursionTemplate {
        // The number of points on the biggest subsimplex
        let max_num_points      = dim + 1;

        let mut pts             = ~[]; // the result
        let mut offsets         = ~[];
        let mut sub_determinants   = ~[];

        // the beginning of the last subsimplices list
        let mut last_dim_begin  = 0;

        // the end of the last subsimplices list
        let mut last_dim_end    = dim + 1 + 1;

        // the number of points of the last subsimplices
        let mut last_num_points = dim + 1;

        let mut map             = TreeMap::<~[uint], uint>::new();

        let mut determinant_index  = 0;

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
                        Some(&v) => sub_determinants.push(v),
                        None     => {
                            for &e in sublist.iter() {
                                pts.push(e);
                                num_added = num_added + 1;
                            }
                            sub_determinants.push(determinant_index);
                            map.insert(sublist, determinant_index);
                            determinant_index = determinant_index + 1;
                        }
                    }
                }

                let mut parent = ~[];
                for k in range(0u, last_num_points + 1) {
                    parent.push(pts[curr + k])
                }


                match map.find(&parent) {
                    Some(&p) => sub_determinants.push(p),
                    None => {
                        sub_determinants.push(determinant_index);
                        // There is no need to keep a place for the full simplex determinant.
                        // So we dont increase the determinant buffer index for the first
                        // iteration.
                        determinant_index = determinant_index + if i == 0 { 0 } else { 1 };
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

        // determinant indices for leaves
        for i in range(0u, max_num_points) {
            sub_determinants.push(*map.find(&~[max_num_points - 1 - i]).unwrap())
        }

        // end to begin offsets
        offsets.unshift(0u);
        offsets.reverse();
        offsets.pop();

        let rev_offsets = offsets.map(|&e| pts.len() - e);
        let num_leaves = rev_offsets[0];

        // reverse points and detereminants
        pts.reverse();
        sub_determinants.reverse();

        // remove the full simplex
        let num_pts = pts.len();
        pts.truncate(num_pts - max_num_points - 1);
        sub_determinants.truncate(num_pts - max_num_points - 1);

        RecursionTemplate {
            offsets:          rev_offsets,
            permutation_list: pts,
            num_determinants: sub_determinants[0] + 1,
            sub_determinants: sub_determinants,
            num_leaves:       num_leaves
        }
    }
}

impl<N: Clone + Zero, V: Dim>
JohnsonSimplex<N, V> {
    /// Creates a new, empty, johnson simplex.
    pub fn new(recursion: Arc<~[RecursionTemplate]>) -> JohnsonSimplex<N, V> {
        let _dim = na::dim::<V>();

        JohnsonSimplex {
            points:             vec::with_capacity(_dim + 1),
            exchange_points:    vec::with_capacity(_dim + 1),
            determinants:       vec::from_elem(recursion.get()[_dim].num_determinants, Zero::zero()),
            recursion_template: recursion
        }
    }

    /// Creates a new, empty johnson simplex. The recursion template uses the thread-local one.
    pub fn new_w_tls() -> JohnsonSimplex<N, V> {

        let recursion = local_data::get(KEY_RECURSION_TEMPLATE, |r| r.map(|rec| rec.clone()));

        match recursion {
            Some(r) => {
                if r.get().len() > na::dim::<V>() {
                    return JohnsonSimplex::new(r)
                }
            },
            _ => { }
        }

        let new_recursion = RecursionTemplate::new(na::dim::<V>());
        local_data::set(KEY_RECURSION_TEMPLATE, new_recursion.clone());
        JohnsonSimplex::new(new_recursion)

    }
}

impl<N: Ord + Clone + Num + Bounded,
     V: Clone + AlgebraicVec<N>>
JohnsonSimplex<N, V> {
    fn do_project_origin(&mut self, reduce: bool) -> V {
        if self.points.is_empty() {
            fail!("Cannot project the origin on an empty simplex.")
        }

        if self.points.len() == 1 {
            return self.points[0].clone();
        }

        let max_num_pts      = self.points.len();
        let recursion        = &self.recursion_template.get()[max_num_pts - 1];
        let mut curr_num_pts = 1u;
        let mut curr         = max_num_pts;

        for c in self.determinants.mut_slice_from(recursion.num_determinants - max_num_pts).mut_iter() {
            *c = One::one();
        }

        // NOTE: Please read that before thinking all those `unsafe_whatever` should be bannished.
        // The whole point of having this `recursion_template` stuff is to speed up the
        // computations having exact precomputed indices.
        // Using safe accesses to vectors kind of makes this useless sinces each array access will
        // be much slower.
        // That is why we use unsafe indexing here. Nothing personal, just a huge need of
        // performances :p
        // There might be a whay to to this nicely with iterators. But indexing is verry intricate
        // here…

        /*
         * first loop: compute all the determinants
         */
        for &end in recursion.offsets.slice_from(2).iter() { // FIXME: try to transform this using a `window_iter()`
            // for each sub-simplex ...
            while (curr != end) { // FIXME: replace this `while` by a `for` when a range with custom increment exist
                unsafe {
                    let mut determinant: N = Zero::zero();
                    let kpt = self.points.unsafe_get(recursion.permutation_list.unsafe_get(curr + 1u)).clone();
                    let jpt = self.points.unsafe_get(recursion.permutation_list.unsafe_get(curr)).clone();

                    // ... with curr_num_pts points ...
                    for i in range(curr + 1, curr + 1 + curr_num_pts) {
                        // ... compute its determinant.
                        let i_pid = recursion.permutation_list.unsafe_get(i);
                        let sub_determinant = self.determinants.unsafe_get(
                                             recursion.sub_determinants.unsafe_get(i)).clone();
                        let delta = sub_determinant * na::sub_dot(&kpt, &jpt, &self.points.unsafe_get(i_pid));

                        determinant = determinant + delta;
                    }

                    self.determinants.unsafe_set(recursion.sub_determinants.unsafe_get(curr), determinant);

                    curr = curr + curr_num_pts + 1; // points + removed point + determinant id
                }
            }

            curr_num_pts = curr_num_pts + 1;
        }

        /*
         * second loop: find the subsimplex containing the projection
         */
        let mut offsets_iter = recursion.offsets.rev_iter();
        offsets_iter.next(); // skip the first offset
        for &end in offsets_iter {
            // for each sub-simplex ...
            while (curr != end) {
                let mut foundit = true;

                // ... with curr_num_pts points permutations ...
                for i in range(0u, curr_num_pts) {
                    unsafe {
                        // ... see if its determinant is positive
                        let det_id = curr - (i + 1) * curr_num_pts;
                        let det    = self.determinants.unsafe_get(recursion.sub_determinants.unsafe_get(det_id));

                        if det > Zero::zero() {
                            // invalidate the children determinant
                            if curr_num_pts > 1 {
                                let subdetid = recursion.sub_determinants.unsafe_get(det_id + 1);

                                if self.determinants.unsafe_get(subdetid) > Zero::zero() {
                                    self.determinants.unsafe_set(subdetid, Bounded::max_value())
                                }
                            }

                            // dont concider this sub-simplex if it has been invalidated by its
                            // parent(s)
                            if det == Bounded::max_value() {
                                foundit = false
                            }
                        }
                        else {
                            // we found a negative determinant: no projection possible here
                            foundit = false
                        }
                    }
                }

                if foundit {
                    // we found a projection!
                    // re-run the same iteration but, this time, compute the projection
                    let mut total_det: N = Zero::zero();
                    let mut proj: V      = Zero::zero();

                    unsafe {
                        for i in range(0u, curr_num_pts) { // FIXME: change this when decreasing loops are implemented
                            // ... see if its determinant is positive
                            let id    = curr - (i + 1) * curr_num_pts;
                            let det   = self.determinants
                                            .unsafe_get(recursion.sub_determinants.unsafe_get(id));

                            total_det = total_det + det;
                            proj = proj +
                                self.points.unsafe_get(recursion.permutation_list
                                                                .unsafe_get(id)) * det;
                        }

                        if reduce {
                            // we need to reduce the simplex 
                            for i in range(0u, curr_num_pts) {
                                let id = curr - (i + 1) * curr_num_pts;
                                self.exchange_points.push(
                                    self.points.unsafe_get(
                                        recursion.permutation_list.unsafe_get(id)));
                            }

                            util::swap(&mut self.exchange_points, &mut self.points);
                            self.exchange_points.clear();
                        }
                    }

                    return proj / total_det;
                }

                curr = curr - curr_num_pts * curr_num_pts;
            }

            curr_num_pts = curr_num_pts - 1;
        }

        Zero::zero()
    }
}

impl<N: Ord + Clone + Num + Bounded + Algebraic, V: Clone + AlgebraicVec<N>>
Simplex<N, V> for JohnsonSimplex<N, V> {
    #[inline]
    fn reset(&mut self, pt: V) {
        self.points.clear();
        self.points.push(pt);
    }

    #[inline]
    fn dimension(&self) -> uint {
        self.points.len() - 1
    }

    #[inline]
    fn max_sq_len(&self) -> N {
        self.points.iter().map(|v| na::sqnorm(v)).max().unwrap()
    }

    #[inline]
    fn contains_point(&self, pt: &V) -> bool {
        self.points.iter().any(|v| pt == v)
    }

    #[inline]
    fn add_point(&mut self, pt: V) {
        self.points.push(pt);
        assert!(self.points.len() <= na::dim::<V>() + 1);
    }

    #[inline]
    fn project_origin_and_reduce(&mut self) -> V {
        self.do_project_origin(true)
    }

    #[inline]
    fn project_origin(&mut self) -> V {
        self.do_project_origin(false)
    }
}

impl ToStr for RecursionTemplate {
    fn to_str(&self) -> ~str {
        let mut res  = ~"RecursionTemplate { ";
        let mut curr = self.num_leaves;
        let mut dim  = 1;

        res = res + "num_determinants: " + self.num_determinants.to_str();

        let mut recursion_offsets_skip_1 = self.offsets.iter();
        recursion_offsets_skip_1.next(); // Skip the two first entries

        for &off in recursion_offsets_skip_1 {
            while (curr != off) {
                res = res + "\n(@" + self.sub_determinants[curr].to_str() + " -> ";

                for i in range(0u, dim) {
                    res = res + self.permutation_list[i + curr].to_str();
                    if i != dim - 1 {
                        res = res + " ";
                    }
                }

                res = res + " - ";

                for i in range(1u, dim) {
                    res = res + self.sub_determinants[i + curr].to_str();
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

#[cfg(test)]
mod test {
    use super::{JohnsonSimplex, RecursionTemplate};
    use nalgebra::na::Vec3;
    use extra::test::BenchHarness;

    #[bench]
    fn bench_johnson_simplex(bh: &mut BenchHarness) {
        let a = Vec3::new(-0.5, -0.5, -0.5);
        let b = Vec3::new(0.0, 0.5, 0.0);
        let c = Vec3::new(0.5, -0.5, -0.5);
        let d = Vec3::new(0.0, -0.5, -0.5);
        let recursion = RecursionTemplate::new(3);

        do bh.iter {
            let mut spl = JohnsonSimplex::new(recursion.clone());

            do 1000.times {
                spl.reset(a);

                spl.add_point(b);
                spl.add_point(c);
                spl.add_point(d);

                spl.project_origin_and_reduce();
            }
        }
    }

    #[bench]
    fn bench_johnson_simplex_tls(bh: &mut BenchHarness) {
        let a = Vec3::new(-0.5, -0.5, -0.5);
        let b = Vec3::new(0.0, 0.5, 0.0);
        let c = Vec3::new(0.5, -0.5, -0.5);
        let d = Vec3::new(0.0, -0.5, -0.5);
        let _ = JohnsonSimplex::<f64, Vec3<f64>>::new_w_tls();

        do bh.iter {
            let mut spl = JohnsonSimplex::new_w_tls();

            do 1000.times {
                spl.reset(a);

                spl.add_point(b);
                spl.add_point(c);
                spl.add_point(d);

                spl.project_origin_and_reduce();
            }
        }
    }
}
