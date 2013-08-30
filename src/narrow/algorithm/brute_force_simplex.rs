#[doc(hidden)];

use std::num::{Zero, One};
use nalgebra::dmat::zero_mat_with_dim;
use nalgebra::traits::inv::Inv;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::vector::AlgebraicVec;
use narrow::algorithm::simplex::Simplex;

pub struct BruteForceSimplex<N, V> {
    points: ~[V]
}

impl<N: Ord + Clone + Num + Algebraic, V: Clone + AlgebraicVec<N>>
BruteForceSimplex<N, V> {
    pub fn new() -> BruteForceSimplex<N, V> {
        BruteForceSimplex { points: ~[] }
    }

    pub fn add_point(&mut self, pt: V) {
        self.points.push(pt)
    }

    fn project_on_subsimplex(points: &[V]) -> Option<V> {
        let     _0: N = Zero::zero();
        let     _1: N = One::one();
        let     dim   = points.len();
        let mut mat   = zero_mat_with_dim(dim);

        for i in range(0u, dim) {
            mat.set(0u, i, &_1)
        }

        for i in range(1u, dim) {
            for j in range(0u, dim) {
                mat.set(
                    i,
                    j,
                    &points[i].sub_dot(&points[0], &points[j])
                )
            }
        }

        if !mat.inplace_inverse() {
            None
        }
        else {
            let mut res: V        = Zero::zero();
            let mut normalizer: N = Zero::zero();

            for i in range(0u, dim) {
                if mat.at(i, 0u) > _0 {
                    let offset = mat.at(i, 0u);
                    res        = res + points[i] * offset;
                    normalizer = normalizer + offset;
                }
                else {
                    return None
                }
            }

            Some(res / normalizer)
        }
    }

    fn project_on_subsimplices(points: ~[V]) -> (V, ~[V]) {
        if points.len() == 1 {
            (points[0].clone(), points)
        }
        else {
            let mut bestproj = BruteForceSimplex::project_on_subsimplex(points);
            let mut bestpts  = points.clone();

            for i in range(0u, points.len()) {
                let mut subsimplex = ~[];
                for j in range(0u, points.len()) {
                    if i != j {
                        subsimplex.push(points[j].clone())
                    }
                }

                let (proj, sub_p_pts) = BruteForceSimplex::project_on_subsimplices(subsimplex);

                match bestproj {
                    Some(ref p) => if p.norm() > proj.norm() {
                        bestpts = sub_p_pts
                    },
                    None    => bestpts = sub_p_pts
                }

                bestproj = match bestproj {
                    Some(ref p) => if p.norm() > proj.norm() {
                        Some(proj)
                    }
                    else {
                        bestproj.clone()
                    },
                    None    => Some(proj)
                }
            }

            (bestproj.unwrap(), bestpts)
        }
    }

    pub fn do_project_origin(&mut self, reduce: bool) -> V {
        let (res, reduction) = BruteForceSimplex::project_on_subsimplices(self.points.clone());

        if reduce {
            self.points = reduction
        }

        res
    }
}

impl<N: Ord + Clone + Num + Algebraic, V: Clone + AlgebraicVec<N>>
Simplex<N, V> for BruteForceSimplex<N, V> {
    fn reset(&mut self, initial_point: V) {
        self.points.clear();
        self.points.push(initial_point);
    }

    fn dimension(&self) -> uint {
        self.points.len() - 1
    }

    fn max_sq_len(&self) -> N {
        self.points.iter().map(|v| v.sqnorm()).max().unwrap()
    }

    fn contains_point(&self, pt: &V) -> bool {
        self.points.iter().any(|v| pt == v)
    }

    fn add_point(&mut self, pt: V) {
        assert!(self.points.len() <= Dim::dim(None::<V>));
        self.points.push(pt)
    }

    fn project_origin_and_reduce(&mut self) -> V {
        self.do_project_origin(true)
    }

    fn project_origin(&mut self) -> V {
        if self.points.is_empty() {
            fail!("Cannot project the origin on an empty simplex.")
        }

        self.do_project_origin(false)
    }
}
