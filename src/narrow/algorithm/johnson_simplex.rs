use std::util;
use std::uint;
use std::num::{Zero, One};
use std::iterator::IteratorUtil;
use std::vec;
use std::local_data;
use extra::treemap::TreeMap;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::sub_dot::SubDot;
use nalgebra::traits::scalar_op::{ScalarMul, ScalarDiv};
use narrow::algorithm::simplex::Simplex;

#[deriving(Eq, ToStr, Clone)]
pub struct JohnsonSimplex<V, N>
{
  priv points:              ~[V],
  priv exchange_points:     ~[V],
  priv cofactors:           ~[N]
}

#[deriving(Eq, Clone)]
struct RecursionTemplate
{
  permutation_list: ~[uint],
  offsets:          ~[uint],
  sub_cofactors:    ~[uint],
  num_cofactors:    uint,
  num_leaves:       uint // useful only for printing…
}

static key_recursion_template: local_data::Key<@mut ~[@RecursionTemplate]> = &local_data::Key;

impl<V: Clone + SubDot<N> + ScalarMul<N> + ScalarDiv<N> + Zero + Add<V, V> + Dim,
     N: Ord + Clone + Eq + DivisionRing + Ord + Bounded>
JohnsonSimplex<V, N>
{
  pub fn new(initial_point: V) -> JohnsonSimplex<V, N>
  {
    let _dim       = Dim::dim::<V>();
    let expoints   = vec::with_capacity(_dim);
    let mut points = vec::with_capacity(_dim);

    JohnsonSimplex::make_permutation_lists::<V, N>();

    points.push(initial_point);

    let res = JohnsonSimplex {
      points:            points
      , exchange_points: expoints
      , cofactors:       vec::from_elem(
        do local_data::get(key_recursion_template) |lopt|
        {
          match lopt
          {
            Some(l) => l[_dim].num_cofactors,
            None    => fail!("Recursion template was not intialized.")
          }
        }
      , Zero::zero())
    };

    res
  }

  fn make_permutation_lists()
  {
    do local_data::get(key_recursion_template) |lopt|
    {
      if lopt.is_none()
      {
        local_data::set(key_recursion_template, @mut ~[])
      }
    }

    do local_data::get(key_recursion_template) |lopt|
    {
      let template = lopt.unwrap();

      for uint::iterate(template.len(), Dim::dim::<V>() + 1u) |dim|
      { template.push(@JohnsonSimplex::make_permutation_list::<V, N>(dim)) }
    }
  }

  // This is the tricky part of the algorithm. This generates all datas needed
  // to run the johson subalgorithm fastly. This should _not_ be run every time
  // the algorithm is executed. Instead, it should be pre-computed, or computed
  // only once for all. The resulting GC-managed list is intented to be shared
  // between all other simplicis with the same dimension.
  fn make_permutation_list(dim: uint) -> RecursionTemplate
  {
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

    for uint::iterate(0, max_num_points) |i|
    { pts.push(i) }

    // initially push the whole simplex (will be removed at the end)
    pts.push(0);

    offsets.push(max_num_points + 1); 

    // ... then remove one point each time
    for uint::iterate(0, dim + 1) |i|
    {
      // for each sub-simplex ...
      let mut curr      = last_dim_begin;
      let mut num_added = 0;

      while (curr != last_dim_end)
      {
        // ... iterate on it ...
        for uint::iterate(0, last_num_points) |j|
        {
          // ... and build all the sublist with one last point
          let mut sublist = ~[];

          // then extract the sub-simplex
          for uint::iterate(0, last_num_points) |k|
          {
            // we remove the j'th point
            if pts[curr + j] != pts[curr + k]
            { sublist.push(pts[curr + k]); }
          }

          // keep a trace of the removed point
          sublist.push(pts[curr + j]);

          match map.find(&sublist)
          {
            Some(&v) => sub_cofactors.push(v),
            None     => {
                          for sublist.iter().advance |&e|
                          {
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
        for uint::iterate(0, last_num_points + 1) |k|
        { parent.push(pts[curr + k]) }


        match map.find(&parent)
        {
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
    for uint::iterate(0, max_num_points) |i|
    { sub_cofactors.push(*map.find(&~[max_num_points - 1 - i]).unwrap()) }

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

  pub fn reset(&mut self, pt: V)
  {
    self.points.clear();
    self.points.push(pt);
  }

  fn do_project_origin(&mut self, reduce: bool) -> V
  {
    // FIXME: for optimization: do special case when there are only 1 point ?
    let _0                   = Zero::zero::<N>();
    let _1                   = One::one::<N>();
    let max_num_pts          = self.points.len();
    let recursion            = do local_data::get(key_recursion_template) |template_opt|
                               { template_opt.unwrap()[max_num_pts - 1] };
    let mut curr_num_pts     = 1u;
    let mut curr             = max_num_pts;

    for uint::iterate(0u, max_num_pts) |i|
    { self.cofactors[recursion.num_cofactors - 1 - i] = _1.clone(); }

    /*
     * first loop: compute all the cofactors
     */
    let mut recursion_offsets_skip_2 = recursion.offsets.iter();
    recursion_offsets_skip_2.next(); // Skip the two first entries
    recursion_offsets_skip_2.next();
    for recursion_offsets_skip_2.advance |&end|
    {
      // for each sub-simplex ...
      let mut _i = curr;
      while (_i != end)
      {
        let mut cofactor = Zero::zero::<N>();
        let j_pid        = recursion.permutation_list[curr];
        let k_pid        = recursion.permutation_list[curr + 1u];

        // ... with curr_num_pts points ...
        for uint::iterate(0u, curr_num_pts) |i|
        {
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
    for offsets_iter.advance |&end|
    {
      // for each sub-simplex ...
      let mut _i = curr;
      while (_i != end)
      {
        let mut foundit = true;
        // ... with curr_num_pts points permutations ...
        for uint::iterate(0u, curr_num_pts) |i|
        {
          // ... see if its cofactor is positive
          let cof_id = _i - (i + 1) * curr_num_pts;
          let cof    = self.cofactors[recursion.sub_cofactors[cof_id]].clone();
          if cof > _0
          {
            // invalidate the children cofactor
            if curr_num_pts > 1
            {
              let subcofid = recursion.sub_cofactors[cof_id + 1];

              if self.cofactors[subcofid] > _0
              { self.cofactors[subcofid] = _invalid_cofactor.clone() }
            }

            // dont concider this sub-simplex if it has been invalidated by its
            // parent(s)
            if cof == _invalid_cofactor
            { foundit = false }
          }
          else // we found a negative cofactor: no projection possible here
          { foundit = false }
        }

        if foundit
        {
          // we found a projection!
          // re-run the same iteration but, this time, compute the projection
          let mut total_cof = _0.clone();
          let mut proj      = Zero::zero::<V>();

          for uint::iterate(0u, curr_num_pts) |i|
          {
            // ... see if its cofactor is positive
            let id    = _i - (i + 1) * curr_num_pts;
            let cof   = self.cofactors[recursion.sub_cofactors[id]].clone();

            total_cof = total_cof + cof;
            proj = proj +
              self.points[recursion.permutation_list[id]].scalar_mul(&cof);
          }

          if reduce // we need to reduce the simplex
          {
            // FIXME: is it possible to do that without copy?
            // Maybe with some kind of cross-vector-swap?
            for uint::iterate(0u, curr_num_pts) |i|
            {
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

    util::unreachable();
  }
}

impl<V: Clone         +
        SubDot<N>    +
        ScalarMul<N> +
        ScalarDiv<N> +
        Zero         +
        Add<V, V>    +
        Norm<N>      +
        Eq           +
        Dim,
     N: Ord          +
        Clone        +
        Eq           +
        DivisionRing +
        Ord          +
        Bounded>
     Simplex<V, N> for JohnsonSimplex<V, N>
{
  pub fn new(initial_point: V) -> JohnsonSimplex<V, N>
  { JohnsonSimplex::new(initial_point) }

  pub fn dimension(&self) -> uint
  { self.points.len() - 1 }

  pub fn max_sq_len(&self) -> N
  { self.points.iter().transform(|v| v.sqnorm()).max().unwrap() }

  pub fn contains_point(&self, pt: &V) -> bool
  { self.points.iter().any(|v| pt == v) }

  pub fn add_point(&mut self, pt: V)
  {
    assert!(self.points.len() <= Dim::dim::<V>());
    self.points.push(pt)
  }

  pub fn project_origin_and_reduce(&mut self) -> V
  { self.do_project_origin(true) }

  pub fn project_origin(&mut self) -> V
  { self.do_project_origin(false) }
}

impl ToStr for RecursionTemplate
{
  fn to_str(&self) -> ~str
  {
    let mut res  = ~"RecursionTemplate { ";
    let mut curr = self.num_leaves;
    let mut dim  = 1;

    res = res + "num_cofactors: " + self.num_cofactors.to_str();

    let mut recursion_offsets_skip_1 = self.offsets.iter();
    recursion_offsets_skip_1.next(); // Skip the two first entries

    for recursion_offsets_skip_1.advance |&off|
    {
      while (curr != off)
      {
        res = res + "\n(@" + self.sub_cofactors[curr].to_str() + " -> ";

        for uint::iterate(0u, dim) |i|
        {
          res = res + self.permutation_list[i + curr].to_str();
          if i != dim - 1
          { res = res + " "; }
        }

        res = res + " - ";

        for uint::iterate(1u, dim) |i|
        {
          res = res + self.sub_cofactors[i + curr].to_str();
          if i != dim - 1
          { res = res + " "; }
        }

        res  = res + ")";
        curr = curr + dim;
      }

      dim += 1;
    }

    res = res + " }\n";

    res = res + "offsets: " + self.offsets.to_str();

    res
  }
}
