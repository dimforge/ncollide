//! Penetration depth computation algorithm approximating the Minkowskis sum.

use std::num::{Zero, One};
use nalgebra::na::{Cast, AlgebraicVecExt, Identity, Translation};
use nalgebra::na;
use geom::{Implicit, Reflection, MinkowskiSum, AnnotatedPoint};
use narrow::algorithm::gjk;
use narrow::algorithm::simplex::Simplex;

/// Trait of geometries having prefered sampling directions for the Minkowski sampling algorithm.
/// Those directions are usually the geometry faces normals.
pub trait PreferedSamplingDirections<V, M> {
    /// Applies a function to this geometry with a given transform.
    fn sample(&self, &M, |V| -> ());
}

/// Computes the closest points between two implicit inter-penetrating shapes. Returns None if the
/// shapes are not in penetration. This can be used as a fallback algorithm for the GJK algorithm.
pub fn closest_points<S:  Simplex<N, AnnotatedPoint<V>>,
                      G1: Implicit<N, V, M> + PreferedSamplingDirections<V, M>,
                      G2: Implicit<N, V, M> + PreferedSamplingDirections<V, M>,
                      N:  Clone + Ord + Num + Float + Cast<f32>,
                      V:  AlgebraicVecExt<N> + Clone,
                      M:  One + Translation<V>>(
                      m1:      &M,
                      g1:      &G1,
                      m2:      &M,
                      g2:      &G2,
                      simplex: &mut S)
                      -> Option<(V, V)> {
    let _0: N     = na::zero();
    let _1m: M    = na::one();
    let reflect2  = Reflection::new(g2);
    let cso       = MinkowskiSum::new(m1, g1, m2, &reflect2);

    // find an approximation of the smallest penetration direction
    let mut best_dir: V = Zero::zero();
    let mut min_dist    = Bounded::max_value();

    // FIXME: avoid code duplication for the closure
    g1.sample(m1, |sample: V| {
        let support = cso.support_point(&Identity::new(), &sample);
        let dist    = na::dot(&sample, &support);

        if (dist < min_dist) {
            best_dir = sample;
            min_dist = dist;
        }
    });

    // FIXME: avoid code duplication for the closure
    g2.sample(m2, |sample: V| {
        let support = cso.support_point(&Identity::new(), &sample);
        let dist    = na::dot(&sample, &support);

        if (dist < min_dist) {
            best_dir = sample;
            min_dist = dist;
        }
    });

    // FIXME: avoid code duplication for the closure
    na::sample_sphere(|sample: V| {
        let support = cso.support_point(&Identity::new(), &sample);
        let dist    = na::dot(&sample, &support);

        if (dist < min_dist) {
            best_dir = sample;
            min_dist = dist;
        }
    });

    if min_dist <= _0 {
        return None
    }

    let shift = best_dir * min_dist;

    let tm2 = na::append_translation(m2, &shift);

    simplex.translate_by(&AnnotatedPoint::new(na::zero(), -shift, -shift));

    match gjk::closest_points_without_margin(m1, g1, &tm2, g2, simplex) {
        None => None, // fail!("Internal error: the origin was inside of the Simplex during phase 1."),
        Some((p1, p2)) => {
            // NOTE: at this point, p1 must *not* be concidered as a good contact point for the
            // first object. For example:
            //
            //
            //                               +-------------+
            //                               |             |
            //                               |    obj2     |
            //                       +-------|-----+       |
            //                       |       +-----+-------+
            //                       |    obj1     |
            //                       |             |
            //                       +-------------+
            //
            // May Become after shifting:
            //                                      +-------------+
            //                                      |             |
            //                                      |    obj2     |
            //                                      |             |
            //                                p2 -> x-------------+
            //                       +-------------x <- p1
            //                       |             |
            //                       |    obj1     |
            //                       |             |
            //                       +-------------+
            //
            // Thus, after un-shifting, p1 becomes clearly invalid:
            //
            //                               +-------------+
            //                               |             |
            //                               |    obj2     |
            //                       +-------|-----+ <- p1 |
            //                       | p2 -> +-----+-------+
            //                       |    obj1     |
            //                       |             |
            //                       +-------------+
            let dist_err = na::norm(&(p2 - p1)) - g1.margin() - g2.margin();
            let center   = (p1 + p2) / na::cast(2.0);

            let p2 = center - best_dir * if dist_err > na::zero() { min_dist - dist_err } else { min_dist };

            Some((center, p2))
        }
    }
}

#[cfg(test)]
mod test {
    use super::closest_points;
    use nalgebra::na::Vec2;
    use geom::{Box, AnnotatedPoint};
    use geom;
    use narrow::algorithm::johnson_simplex::JohnsonSimplex;

    #[test]
    fn test_closest_points() {
        let a = Box::new(Vec2::new(5.0f32, 1.0));
        let b = Box::new(Vec2::new(5.0f32, 1.0));
        let ta = Vec2::new(0.0f32, 0.0);
        let tb = Vec2::new(7.0f32, 1.0);
        let mut splx: JohnsonSimplex<f32, AnnotatedPoint<Vec2<f32>>> = JohnsonSimplex::new_w_tls();
        splx.reset(geom::cso_support_point_without_margin(&ta, &a, &tb, &b, Vec2::new(1.0f32, 1.0)));
        println!("{:?}", closest_points(&ta, &a, &tb, &b, &mut splx));
    }
}
