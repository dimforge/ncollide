use core::at_vec::build;
use core::vec::{push, len};
use std::treemap::*;
use nalgebra::traits::dim::Dim;

pub struct JohnsonSimplex<V>
{
  permutation_list: RecursionTemplate,
  points:           ~[~V]
}

struct RecursionTemplate
{
  permutation_list: @[uint],
  offsets:          @[uint],
  sub_cofactors:    @[uint]
}

impl<V: Dim + Copy> JohnsonSimplex<V>
{
  fn _make_permutation_lists() -> RecursionTemplate
  {
    // FIXME: should generate the subsimplices lists too
    JohnsonSimplex::_make_permutation_list::<V>(Dim::dim::<V>())
  }

  fn _make_permutation_list(dim: uint) -> RecursionTemplate
  {
    // FIXME: the permutation list should be computed once for all, at compile
    // time. I dont know how to do that though…

    // the number of points on the biggest subsimplex
    let max_num_points      = dim + 1;

    let mut pts             = ~[];      // the result
    let mut offsets         = ~[];
    let mut sub_cofactors   = ~[];

    // the beginning of the last subsimplices list
    let mut last_dim_begin  = 0;

    // the end of the last subsimplices list
    let mut last_dim_end    = dim + 1;

    // the number of points of the last subsimplices
    let mut last_num_points = dim + 1;

    // initially push the whole simplex ...
    for uint::range(0, max_num_points) |i|
    { push(&mut pts, i) }

    push(&mut offsets,max_num_points); 

    // ... then remove one point each time
    for uint::range(0, dim) |_|
    {
      // for each sub-simplex ...
      let mut curr      = last_dim_begin;
      let mut num_added = 0;
      let mut map       = TreeMap::new::<~[uint], uint>();

      while (curr != last_dim_end)
      {
        // ... iterate on it ...
        for uint::range(0, last_num_points) |j|
        {
          // ... and build all the sublist with one last point
          let mut sublist = ~[];

          for uint::range(0, last_num_points) |k|
          {
            // we remove the k'th point
            if (pts[curr + j] != pts[curr + k])
            { push(&mut sublist, pts[curr + k]); }
          }

          match map.find(&sublist)
          {
            Some(&v) => push(&mut sub_cofactors, v),
            None     => {
                          push(&mut sub_cofactors, len(pts));
                          map.insert(copy sublist, len(pts));
                          for sublist.each |&e|
                          {
                            push(&mut pts, e);
                            num_added += 1;
                          }
                        }
          }

        }

        curr += last_num_points;
      }

      // initialize the next iteration with one less point
      last_dim_begin   = last_dim_end;
      last_dim_end    += num_added;
      push(&mut offsets, last_dim_end);
      last_num_points -= 1;
    }

    println(offsets.to_str());
    println(pts.to_str());
    println(sub_cofactors.to_str());

    let at_permutation_list =
      do build |push_to_managed_vec|
      {
        for pts.each |&e|
        { push_to_managed_vec(e) }
      };

    let at_offsets =
      do build |push_to_managed_vec|
      {
        for offsets.each |&e|
        { push_to_managed_vec(e) }
      };

    let at_sub_cofactors =
      do build |push_to_managed_vec|
      {
        for sub_cofactors.each |&e|
        { push_to_managed_vec(e) }
      };

    RecursionTemplate {
      permutation_list: at_permutation_list,
      offsets:          at_offsets,
      sub_cofactors:    at_sub_cofactors
    }
  }

  pub fn reset(&mut self)
  { self.points.clear() }

  pub fn new(initial_point: &V) -> JohnsonSimplex<V>
  {
    JohnsonSimplex {
      permutation_list: JohnsonSimplex::_make_permutation_lists::<V>()
      , points: ~[~*initial_point]
    }
  }

  /*
  fn project_origin() -> ~JohnsonSimplex<V>
  {
    let offsets; // binomiar coefficient
    let determinants;

    for uint::range(1u, points.size()) |i|
    {
      // iterate on the sub-simplices of dimension i
    }
  }
  */
}

impl<V: ToStr> ToStr for JohnsonSimplex<V>
{
  fn to_str(&self) -> ~str
  { ~"JohnsonSimplex { " + "FIXME" + " }" }
}

impl ToStr for RecursionTemplate
{
  fn to_str(&self) -> ~str
  {
    let mut res  = ~"RecursionTemplate { ";
    let mut curr = 0;
    let mut dim  = self.offsets[0];

    for self.offsets.each |&off|
    {
      while (curr != off)
      {
        res += ~"(@" + curr.to_str() + " -> ";

        for uint::range(0u, dim) |i|
        {
          res += self.permutation_list[i + curr].to_str();
          if (i != dim - 1)
          { res += " "; }
        }

        res  += " - ";

        if (dim != 1)
        {
          for uint::range(0u, dim) |i|
          {
            res += self.sub_cofactors[i + curr].to_str();
            if (i != dim - 1)
            { res += " "; }
          }
        }

        res  += ")";
        curr += dim;
      }

      dim -= 1;
    }

    res += " }";

    res
  }
}
