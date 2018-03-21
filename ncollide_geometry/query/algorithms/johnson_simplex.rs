//!  Simplex using the Johnson subalgorithm to compute the projection of the origin on the simplex.

use std::mem;
use std::iter;
use std::cell::RefCell;
use std::sync::Arc;
use std::collections::BTreeMap;
use num::Bounded;

use na;
use query::algorithms::simplex::Simplex;
use math::Point;

thread_local!(static KEY_RECURSION_TEMPLATE: RefCell<Arc<Vec<JohnsonSimplexTemplate>>> = RefCell::new(Arc::new(Vec::new())));

///  Simplex using the Johnson subalgorithm to compute the projection of the origin on the simplex.
#[derive(Clone)]
pub struct JohnsonSimplex<P: Point> {
    recursion_template: Arc<Vec<JohnsonSimplexTemplate>>,
    points: Vec<P>,
    exchange_points: Vec<P>,
    determinants: Vec<P::Real>,
}

/// Set of indices to explain to the JohnsonSimplex how to do its work.
/// Building this is very time consuming, and thus should be shared between all instances of the
/// Johnson simplex.
#[derive(PartialEq, Clone)]
pub struct JohnsonSimplexTemplate {
    #[doc(hidden)]
    permutation_list: Vec<usize>,
    #[doc(hidden)]
    offsets: Vec<usize>,
    #[doc(hidden)]
    sub_determinants: Vec<usize>,
    #[doc(hidden)]
    num_determinants: usize,
    #[doc(hidden)]
    num_leaves: usize, // useful only for printing…
}

impl JohnsonSimplexTemplate {
    /// Creates a new set of Recursion simplex sharable between any Johnson simplex having a
    /// dimension inferior or equal to `dimension`.
    pub fn new(dimension: usize) -> Arc<Vec<JohnsonSimplexTemplate>> {
        let mut template = Vec::with_capacity(dimension + 1);

        for dimension in 0usize..dimension + 1 {
            template.push(JohnsonSimplexTemplate::make_permutation_list(dimension))
        }

        Arc::new(template)
    }

    // pub fn to_raw_str(&self) -> ~str {
    //     let res = "permutation_list: " + self.permutation_list.to_str() + ", " +
    //               "offset: "           + self.offsets.to_str() + ", " +
    //               "sub_determinants: " + self.sub_determinants.to_str();

    //     res
    // }

    // This is the tricky part of the algorithm. This generates all datas needed to run the johson
    // subalgorithm fastly. This should _not_ be run every time the algorithm is executed. Instead,
    // it should be pre-computed, or computed only once for all. The resulting list is intented to
    // be shared between all other simplices with the same dimension.
    fn make_permutation_list(dimension: usize) -> JohnsonSimplexTemplate {
        // The number of points on the biggest subsimplex
        let max_num_points = dimension + 1;

        let mut pts = Vec::new(); // the result
        let mut offsets = Vec::new();
        let mut sub_determinants = Vec::new();

        // the beginning of the last subsimplices list
        let mut last_dim_begin = 0;

        // the end of the last subsimplices list
        let mut last_dim_end = dimension + 1 + 1;

        // the number of points of the last subsimplices
        let mut last_num_points = dimension + 1;

        let mut map = BTreeMap::<Vec<usize>, usize>::new();

        let mut determinant_index = 0;

        for i in 0..max_num_points {
            pts.push(i)
        }

        // Initially push the whole simplex (will be removed at the end).
        pts.push(0);

        offsets.push(max_num_points + 1);

        // ... then remove one point each time
        for i in 0usize..dimension + 1 {
            // for each sub-simplex ...
            let mut curr = last_dim_begin;
            let mut num_added = 0;

            while curr != last_dim_end {
                // ... iterate on it ...
                for j in 0usize..last_num_points {
                    // ... and build all the sublist with one last point
                    let mut sublist = Vec::new();

                    // then extract the sub-simplex
                    for k in 0usize..last_num_points {
                        // we remove the j'th point
                        if pts[curr + j] != pts[curr + k] {
                            sublist.push(pts[curr + k]);
                        }
                    }

                    // keep a trace of the removed point
                    sublist.push(pts[curr + j]);

                    match map.get(&sublist) {
                        Some(&v) => sub_determinants.push(v),
                        None => {
                            for &e in sublist.iter() {
                                pts.push(e);
                                num_added = num_added + 1;
                            }
                            sub_determinants.push(determinant_index);
                            let _ = map.insert(sublist, determinant_index);
                            determinant_index = determinant_index + 1;
                        }
                    }
                }

                let mut parent = Vec::new();
                for k in 0usize..last_num_points + 1 {
                    parent.push(pts[curr + k])
                }

                match map.get(&parent) {
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
            last_dim_begin = last_dim_end;
            last_dim_end = last_dim_end + num_added;
            offsets.push(last_dim_end);
            last_num_points = last_num_points - 1;
        }

        // determinant indices for leaves
        for i in 0usize..max_num_points {
            sub_determinants.push(*map.get(&vec![max_num_points - 1 - i]).unwrap())
        }

        // end to begin offsets
        offsets.insert(0, 0usize);
        offsets.reverse();
        let _ = offsets.pop();

        let rev_offsets: Vec<usize> = offsets.iter().map(|&e| pts.len() - e).collect();
        let num_leaves = rev_offsets[0];

        // reverse points and detereminants
        pts.reverse();
        sub_determinants.reverse();

        // remove the full simplex
        let num_pts = pts.len();
        pts.truncate(num_pts - max_num_points - 1);
        sub_determinants.truncate(num_pts - max_num_points - 1);

        JohnsonSimplexTemplate {
            offsets: rev_offsets,
            permutation_list: pts,
            num_determinants: sub_determinants[0] + 1,
            sub_determinants: sub_determinants,
            num_leaves: num_leaves,
        }
    }
}

impl<P: Point> JohnsonSimplex<P> {
    /// Creates a new, empty, Johnson simplex.
    pub fn new(recursion: Arc<Vec<JohnsonSimplexTemplate>>) -> JohnsonSimplex<P> {
        let _dimension = na::dimension::<P::Vector>();

        JohnsonSimplex {
            points: Vec::with_capacity(_dimension + 1),
            exchange_points: Vec::with_capacity(_dimension + 1),
            determinants: iter::repeat(na::zero())
                .take(recursion[_dimension].num_determinants)
                .collect(),
            recursion_template: recursion,
        }
    }

    /// Creates a new, empty Johnson simplex. The recursion template uses the thread-local one.
    pub fn new_w_tls() -> JohnsonSimplex<P> {
        KEY_RECURSION_TEMPLATE.with(|rec| {
            if rec.borrow().len() <= na::dimension::<P::Vector>() {
                *rec.borrow_mut() = JohnsonSimplexTemplate::new(na::dimension::<P::Vector>());
            }
            JohnsonSimplex::new(rec.borrow().clone())
        })
    }
}

impl<P: Point> JohnsonSimplex<P> {
    fn do_project_origin(&mut self, reduce: bool) -> P {
        if self.points.is_empty() {
            panic!("Cannot project the origin on an empty simplex.")
        }

        if self.points.len() == 1 {
            return self.points[0];
        }

        let max_num_pts = self.points.len();
        let recursion = &self.recursion_template[max_num_pts - 1];
        let mut curr_num_pts = 1usize;
        let mut curr = max_num_pts;

        let ndets = self.determinants.len();
        for c in self.determinants[recursion.num_determinants - max_num_pts..ndets].iter_mut() {
            *c = na::one();
        }

        // NOTE: The whole point of having the `recursion_template` system is to speed up the
        // computations having exact precomputed indices, hence the unsafe accesses here.
        // There might be a way to do this nicely with iterators. But indexing is verry intricate
        // here…

        /*
         * First loop: compute all the determinants.
         */
        for &end in recursion.offsets[2..].iter() {
            // FIXME: try to transform this using a `window_iter()`
            // For each sub-simplex ...
            while curr != end {
                // FIXME: replace this `while` by a `for` when a range with custom increment exist
                unsafe {
                    let mut determinant: P::Real = na::zero();
                    let kpt = *self.points[..]
                        .get_unchecked(*recursion.permutation_list[..].get_unchecked(curr + 1usize));
                    let jpt = *self.points[..]
                        .get_unchecked(*recursion.permutation_list[..].get_unchecked(curr));

                    // ... with curr_num_pts points ...
                    for i in curr + 1..curr + 1 + curr_num_pts {
                        // ... compute its determinant.
                        let i_pid = *recursion.permutation_list[..].get_unchecked(i);
                        let sub_determinant = *self.determinants[..]
                            .get_unchecked(*recursion.sub_determinants[..].get_unchecked(i));
                        let delta = sub_determinant
                            * na::dot(
                                &(kpt - jpt),
                                &self.points[..].get_unchecked(i_pid).coordinates(),
                            );

                        determinant = determinant + delta;
                    }

                    *self.determinants
                        .as_mut_ptr()
                        .offset(*recursion.sub_determinants[..].get_unchecked(curr) as isize) =
                        determinant;

                    curr = curr + curr_num_pts + 1; // points + removed point + determinant id
                }
            }

            curr_num_pts = curr_num_pts + 1;
        }

        /*
         * Second loop: find the subsimplex containing the projection.
         */
        let mut offsets_iter = recursion.offsets[..].iter().rev();
        let _ = offsets_iter.next(); // skip the first offset
        for &end in offsets_iter {
            // For each sub-simplex ...
            while curr != end {
                let mut foundit = true;

                // ... with curr_num_pts points permutations ...
                for i in 0usize..curr_num_pts {
                    unsafe {
                        // ... see if its determinant is positive.
                        let det_id = curr - (i + 1) * curr_num_pts;
                        let det = *self.determinants[..]
                            .get_unchecked(*recursion.sub_determinants[..].get_unchecked(det_id));

                        if det > na::zero() {
                            // Invalidate the children determinant.
                            if curr_num_pts > 1 {
                                let subdetid =
                                    *recursion.sub_determinants[..].get_unchecked(det_id + 1);

                                if *self.determinants[..].get_unchecked(subdetid) > na::zero() {
                                    *self.determinants.as_mut_ptr().offset(subdetid as isize) =
                                        Bounded::max_value()
                                }
                            }

                            // Don't concider this sub-simplex if it has been invalidated by its
                            // parent(s).
                            if det == Bounded::max_value() {
                                foundit = false
                            }
                        } else {
                            // We found a negative determinant: no projection possible here.
                            foundit = false
                        }
                    }
                }

                if foundit {
                    // We found a projection!
                    // Re-run the same iteration but, this time, compute the projection.
                    let mut total_det: P::Real = na::zero();
                    let mut proj = P::origin();

                    unsafe {
                        for i in 0usize..curr_num_pts {
                            // FIXME: change this when decreasing loops are implemented
                            // ... see if its determinant is positive
                            let id = curr - (i + 1) * curr_num_pts;
                            let det = *self.determinants[..]
                                .get_unchecked(*recursion.sub_determinants[..].get_unchecked(id));

                            total_det = total_det + det;
                            proj.axpy(
                                det,
                                self.points[..]
                                    .get_unchecked(*recursion.permutation_list[..].get_unchecked(id)),
                                na::one(),
                            );
                        }

                        if reduce {
                            // we need to reduce the simplex
                            for i in 0usize..curr_num_pts {
                                let id = curr - (i + 1) * curr_num_pts;
                                self.exchange_points.push(*self.points[..]
                                    .get_unchecked(*recursion.permutation_list[..].get_unchecked(id)));
                            }

                            mem::swap(&mut self.exchange_points, &mut self.points);
                            self.exchange_points.clear();
                        }
                    }

                    return proj / total_det;
                }

                curr = curr - curr_num_pts * curr_num_pts;
            }

            curr_num_pts = curr_num_pts - 1;
        }

        P::origin()
    }
}

impl<P: Point> Simplex<P> for JohnsonSimplex<P> {
    #[inline]
    fn reset(&mut self, pt: P) {
        self.points.clear();
        self.points.push(pt);
    }

    #[inline]
    fn dimension(&self) -> usize {
        self.points.len() - 1
    }

    #[inline]
    fn max_sq_len(&self) -> P::Real {
        let mut max_sq_len = na::zero();

        for p in self.points.iter() {
            let norm = na::norm_squared(&p.coordinates());

            if norm > max_sq_len {
                max_sq_len = norm
            }
        }

        max_sq_len
    }

    #[inline]
    fn contains_point(&self, pt: &P) -> bool {
        self.points.iter().any(|v| pt == v)
    }

    #[inline]
    fn add_point(&mut self, pt: P) -> bool {
        self.points.push(pt);
        assert!(self.points.len() <= na::dimension::<P::Vector>() + 1);
        return true;
    }

    #[inline]
    fn point(&self, i: usize) -> P {
        self.points[i]
    }

    #[inline]
    fn project_origin_and_reduce(&mut self) -> P {
        self.do_project_origin(true)
    }

    #[inline]
    fn project_origin(&mut self) -> P {
        self.do_project_origin(false)
    }

    #[inline(always)]
    fn modify_pnts(&mut self, f: &Fn(&mut P)) {
        for pt in self.points.iter_mut() {
            f(pt)
        }
    }
}
