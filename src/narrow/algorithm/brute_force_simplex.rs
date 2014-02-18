#[doc(hidden)];

use nalgebra::na::{DMat, Inv, FloatVec};
use nalgebra::na;
use narrow::algorithm::simplex::Simplex;
use math::N;

#[deriving(Encodable, Decodable)]
pub struct BruteForceSimplex<_V> {
    points: ~[_V]
}

impl<_V: Clone + FloatVec<N>>
BruteForceSimplex<_V> {
    pub fn new() -> BruteForceSimplex<_V> {
        BruteForceSimplex { points: ~[] }
    }

    pub fn add_point(&mut self, pt: _V) {
        self.points.push(pt)
    }

    fn project_on_subsimplex(points: &[_V]) -> Option<_V> {
        let     _0: N = na::zero();
        let     _1: N = na::one();
        let     dim   = points.len();
        let mut mat   = DMat::new_zeros(dim, dim);

        for i in range(0u, dim) {
            mat.set(0u, i, _1.clone())
        }

        for i in range(1u, dim) {
            for j in range(0u, dim) {
                mat.set(
                    i,
                    j,
                    na::sub_dot(&points[i], &points[0], &points[j])
                )
            }
        }

        if !mat.inv() {
            None
        }
        else {
            let mut res: _V       = na::zero();
            let mut normalizer: N = na::zero();

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

    fn project_on_subsimplices(points: ~[_V]) -> (_V, ~[_V]) {
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
                    Some(ref p) => if na::norm(p) > na::norm(&proj) {
                        bestpts = sub_p_pts
                    },
                    None    => bestpts = sub_p_pts
                }

                bestproj = match bestproj {
                    Some(ref p) => if na::norm(p) > na::norm(&proj) {
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

    pub fn do_project_origin(&mut self, reduce: bool) -> _V {
        let (res, reduction) = BruteForceSimplex::project_on_subsimplices(self.points.clone());

        if reduce {
            self.points = reduction
        }

        res
    }
}

impl<_V: Clone + FloatVec<N>>
Simplex<_V> for BruteForceSimplex<_V> {
    #[inline]
    fn reset(&mut self, initial_point: _V) {
        self.points.clear();
        self.points.push(initial_point);
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
    fn contains_point(&self, pt: &_V) -> bool {
        self.points.iter().any(|v| pt == v)
    }

    #[inline]
    fn add_point(&mut self, pt: _V) {
        assert!(self.points.len() <= na::dim::<_V>());
        self.points.push(pt)
    }

    #[inline]
    fn project_origin_and_reduce(&mut self) -> _V {
        self.do_project_origin(true)
    }

    #[inline]
    fn project_origin(&mut self) -> _V {
        if self.points.is_empty() {
            fail!("Cannot project the origin on an empty simplex.")
        }

        self.do_project_origin(false)
    }

    #[inline]
    fn translate_by(&mut self, v: &_V) {
        for p in self.points.mut_iter() {
            *p = *p + *v;
        }
    }
}
