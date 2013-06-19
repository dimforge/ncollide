use std::uint::iterate;
use std::at_vec::build;
use std::num::{Zero, One};
use std::iterator::IteratorUtil;
use std::vec;
use extra::treemap::TreeMap;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::sub_dot::SubDot;
use nalgebra::traits::scalar_op::{ScalarMul, ScalarDiv};

#[deriving(ToStr, Eq)]
pub struct JohnsonSimplex<V, T>
{
  recursion_templates: @[RecursionTemplate],
  points:              ~[~V],
  cofactors:           ~[T]
}

#[deriving(Eq)]
struct RecursionTemplate
{
  permutation_list: ~[uint],
  offsets:          ~[uint],
  sub_cofactors:    ~[uint],
  cof_to_simplex:   ~[(uint, uint)], // (offset, size)
  num_cofactors:    uint,
  num_leaves:       uint // useful only for printing…

}

// FIXME: remove the ToStr trait constraint.
impl<V: Copy + SubDot<T> + ScalarMul<T> + ScalarDiv<T> + Zero + Add<V, V> + Dim
+ ToStr,
     T: Ord + Copy + Clone + Eq + DivisionRing + Ord + ToStr>
JohnsonSimplex<V, T>
{
  fn _make_permutation_lists() -> @[RecursionTemplate]
  {
    do build |push_to_at_vec|
    {
      for iterate(0u, Dim::dim::<V>() + 1u) |dim|
      {
        push_to_at_vec(
          JohnsonSimplex::_make_permutation_list::<V, T>(dim))
      }
    }
  }

  // This is the tricky part of the algorithm. This generates all datas needed
  // to run the johson subalgorithm fastly. This should _not_ be run every time
  // the algorithm is executed. Instead, it should be pre-computed, or computed
  // only once for all. The resulting GC-managed list is intented to be shared
  // between all other simplicis with the same dimension.
  fn _make_permutation_list(dim: uint) -> RecursionTemplate
  {
    // FIXME: the permutation list should be computed once for all, at compile
    // time. I dont know how to do that though…

    // The number of points on the biggest subsimplex
    let max_num_points      = dim + 1;

    let mut pts             = ~[]; // the result
    let mut offsets         = ~[];
    let mut sub_cofactors   = ~[];
    let mut cof_to_simplex  = ~[];

    // the beginning of the last subsimplices list
    let mut last_dim_begin  = 0;

    // the end of the last subsimplices list
    let mut last_dim_end    = dim + 1 + 1;

    // the number of points of the last subsimplices
    let mut last_num_points = dim + 1;

    let mut map             = TreeMap::new::<~[uint], uint>();

    let mut cofactor_index  = 0;

    for iterate(0, max_num_points) |i|
    { pts.push(i) }

    // initially push the whole simplex (will be removed at the end)
    pts.push(0);

    offsets.push(max_num_points + 1); 

    // ... then remove one point each time
    for iterate(0, dim + 1) |i|
    {
      // for each sub-simplex ...
      let mut curr      = last_dim_begin;
      let mut num_added = 0;

      while (curr != last_dim_end)
      {
        // ... iterate on it ...
        for iterate(0, last_num_points) |j|
        {
          // ... and build all the sublist with one last point
          let mut sublist = ~[];

          // then extract the sub-simplex
          for iterate(0, last_num_points) |k|
          {
            // we remove the j'th point
            if (pts[curr + j] != pts[curr + k])
            { sublist.push(pts[curr + k]); }
          }

          // keep a trace of the removed point
          sublist.push(pts[curr + j]);

          match map.find(&sublist)
          {
            Some(&v) => sub_cofactors.push(v),
            None     => {
                          for sublist.each |&e|
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
        for iterate(0, last_num_points + 1) |k|
        { parent.push(pts[curr + k]) }


        match map.find(&parent)
        {
          Some(&p) => {
            cof_to_simplex.push((curr - max_num_points, last_num_points));
            sub_cofactors.push(p)
          },
          None => {
            cof_to_simplex.push((curr - max_num_points, last_num_points));
            sub_cofactors.push(cofactor_index);
            // There is no need to keep a place for the full simplex cofactor.
            // So we dont increase the cofactor buffer index for the first
            // iteration.
            cofactor_index = cofactor_index + if (i == 0) { 0 } else { 1 };
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
    for iterate(0, max_num_points) |i|
    { sub_cofactors.push(*map.find(&~[max_num_points - 1 - i]).unwrap()) }

    vec::reverse(pts);
    vec::reverse(sub_cofactors);
    vec::unshift(&mut offsets, 0u);
    vec::pop(&mut offsets);
    vec::reverse(offsets);
    vec::pop(&mut offsets);
    let mut rev_offsets = vec::map(offsets, |&e| pts.len() - e);
    let num_leaves = vec::shift(&mut rev_offsets);

    vec::shift(&mut cof_to_simplex);

    // remove the full simplex
    let num_pts = pts.len();
    let reverse_cof_to_simplex =
      vec::map(cof_to_simplex, |&(id, sz)| (num_pts - max_num_points - id - 1, sz));
    vec::truncate(&mut pts, num_pts - max_num_points - 1);
    vec::truncate(&mut sub_cofactors, num_pts - max_num_points - 1);

    RecursionTemplate {
      offsets:          rev_offsets,
      permutation_list: pts,
      num_cofactors:    sub_cofactors[0] + 1,
      sub_cofactors:    sub_cofactors,
      cof_to_simplex:   reverse_cof_to_simplex,
      num_leaves:       num_leaves
    }
  }

  pub fn reset(&mut self)
  { self.points.clear() }

  pub fn new(initial_point: &V) -> JohnsonSimplex<V, T>
  {
    let perm_list = JohnsonSimplex::_make_permutation_lists::<V, T>();

    JohnsonSimplex {
      recursion_templates: perm_list
      , points:            ~[~copy *initial_point]
      , cofactors:         vec::from_elem(
                             perm_list[Dim::dim::<V>()].num_cofactors,
                             Zero::zero())
    }
  }

  pub fn add_point(&mut self, pt: &V)
  { vec::push(&mut self.points, ~copy *pt) }

  pub fn project_origin(&mut self) -> Option<V> // FIXME: ~JohnsonSimplex<V>
  {
    // FIXME: do special case when there are only 1 point

    let _0                   = Zero::zero::<T>();
    let _1                   = One::one::<T>();
    let _2                   = copy _1 + copy _1;
    let max_num_pts          = self.points.len();
    let recursion            = &self.recursion_templates[max_num_pts - 1];
    let mut curr_num_pts     = 1u;
    let mut curr             = max_num_pts;

    for iterate(0u, max_num_pts) |i|
    { self.cofactors[recursion.num_cofactors - 1 - i] = copy _1; }

    /*
     * first loop: compute all the cofactors
     */
    for recursion.offsets.each() |&end|
    {
      // for each sub-simplex ...
      // for iterate_step(curr, end, curr_num_pts as int + 1) |_|
      let mut _i = curr;
      while (_i != end)
      {
        let mut cofactor = Zero::zero::<T>();
        let j_pid        = recursion.permutation_list[curr];
        let k_pid        = recursion.permutation_list[curr + 1u];
        let mut invalid  = false;

        // ... with curr_num_pts points ...
        for iterate(0u, curr_num_pts) |i|
        {
          // ... compute its cofactor.
          let i_pid        = recursion.permutation_list[curr + 1 + i];
          let sub_cofactor = copy self.cofactors[recursion.sub_cofactors[curr + 1 + i]];

          if (sub_cofactor < _0)
          {
            // there is a negative sub-cofactor: propagate the sign
            invalid = true;
            break;
          }

          cofactor = cofactor + sub_cofactor *
                      self.points[k_pid].sub_dot(self.points[j_pid],
                                                 self.points[i_pid]);
        }

        if (invalid)
        { cofactor = -_1; }
        else if (cofactor < _0)
        {
          // special value meaning this is the first negative cofactors
          cofactor = -_2;
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
    println(self.cofactors.to_str());
    if (self.cofactors[0] >= _0)
    { None } // the origin is inside of the simplex
    else
    {
      for self.cofactors.rev_iter().enumerate().advance |(i, &cof)|
      {
        if (cof == -_2)
        {
          // We found the good sub-simplex.
          let (subsimplex_ids, subsimplex_size) = recursion.cof_to_simplex[i];

          let mut tot_cofactor = Zero::zero::<T>();
          let mut proj         = Zero::zero::<V>();

          let mut i = subsimplex_ids;
          // for iterate_rev(subsimplex_ids, subsimplex_ids - subsimplex_size) |i|
          while (i != subsimplex_ids - subsimplex_size)
          {
            // FIXME: build the sub-simplex here
            let sub_cofactor = copy self.cofactors[recursion.sub_cofactors[i]];
            tot_cofactor     = tot_cofactor + copy sub_cofactor;
            println(~"__ADDED: " +
                    self.points[recursion.permutation_list[i]].to_str());
            println(~"__at: " + sub_cofactor.to_str());
            proj             = proj + 
              self.points[recursion.permutation_list[i]].scalar_mul(&sub_cofactor);

            i = - 1;
          }

          proj.scalar_div_inplace(&tot_cofactor);

          return Some(proj);
        }
      }

      fail!("Internal error: no projection found while the origin is outside.");
    }
  }
}

impl ToStr for RecursionTemplate
{
  fn to_str(&self) -> ~str
  {
    let mut res  = ~"RecursionTemplate { ";
    let mut curr = self.num_leaves;
    let mut dim  = 2; // self.offsets[0];

    res += ~"num_cofactors: " + self.num_cofactors.to_str() + "\n";
    res += ~"< " + curr.to_str() + " leaves not displayed >";

    for self.offsets.each |&off|
    {
      while (curr != off)
      {
        res += ~"\n(@" + self.sub_cofactors[curr].to_str() + " -> ";

        for iterate(0u, dim) |i|
        {
          res += self.permutation_list[i + curr].to_str();
          if (i != dim - 1)
          { res += " "; }
        }

        res  += " - ";

        for iterate(1u, dim) |i|
        {
          res += self.sub_cofactors[i + curr].to_str();
          if (i != dim - 1)
          { res += " "; }
        }

        res  += ")";
        curr += dim;
      }

      dim += 1;
    }

    res += " }\n";

    res += ~"offsets: " + self.offsets.to_str();

    res
  }
}
