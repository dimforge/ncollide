use nalgebra::na;
use nalgebra::na::{Cast, AlgebraicVecExt, VecExt, Transform, Rotate, Identity};
use geom::subsimplex_mesh::{Subsimplex, SubsimplexMesh};
use narrow::algorithm::johnson_simplex::JohnsonSimplex;
use ray::{Ray, RayCast, RayCastWithTransform, gjk_toi_and_normal_with_ray};
use partitioning::bvt_visitor::RayInterferencesCollector;

impl<'a,
     N: Ord + Num + Float + Cast<f32>,
     V: AlgebraicVecExt<N> + Clone>
RayCast<N, V> for Subsimplex<'a, N, V> {
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        if self.margin().is_zero() {
            match na::dim::<V>() {
                2 => {
                    let a = &self.vertices()[self.indices()[0]];
                    let b = &self.vertices()[self.indices()[1]];
                    segment_toi_and_normal_with_ray(a, b, ray)
                }
                3 => {
                    let a = &self.vertices()[self.indices()[0]];
                    let b = &self.vertices()[self.indices()[1]];
                    let c = &self.vertices()[self.indices()[2]];
                    triangle_toi_and_normal_with_ray(a, b, c, ray)
                }
                _ => gjk_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, V>::new_w_tls(), ray)
            }
        }
        else {
            gjk_toi_and_normal_with_ray(&Identity::new(), self, &mut JohnsonSimplex::<N, V>::new_w_tls(), ray)
        }
    }
}

pub fn triangle_toi_and_normal_with_ray<N: Num + Ord + Signed + Algebraic,
                                        V: AlgebraicVecExt<N>>(
                                        a:   &V,
                                        b:   &V,
                                        c:   &V,
                                        ray: &Ray<V>)
                                        -> Option<(N, V)> {
    assert!(na::dim::<V>() == 3);

    /*
     * NOTE: redefine the cross product in order to not require the `Cross` trait.
     */
    #[inline(always)]
    fn cross<N: Num, V: VecExt<N>>(a: &V, b: &V) -> V {
        let mut res = na::zero::<V>();

        res.set(0, a.at(1) * b.at(2) - a.at(2) * b.at(1));
        res.set(1, a.at(2) * b.at(0) - a.at(0) * b.at(2));
        res.set(2, a.at(0) * b.at(1) - a.at(1) * b.at(0));

        res
    }

    let ab = *b - *a;
    let ac = *c - *a;

    // normal
    let n = cross(&ab, &ac);
    let d = na::dot(&n, &ray.dir);

    // the normal and the ray direction are parallel
    if d.is_zero() {
        return None;
    }

    let ap = ray.orig - *a;
    let t  = na::dot(&ap, &n);

    // the ray does not intersect the plane defined by the triangle
    if (t < na::zero() && d < na::zero()) || 
       (t > na::zero() && d > na::zero()) {
        return None;
    }

    let d = d.abs();

    //
    // intersection: compute barycentric coordinates
    //
    let e = -cross(&ray.dir, &ap);
    let v = na::dot(&ac, &e);

    if (v < na::zero() || v > d) {
        return None;
    }

    let w = -na::dot(&ab, &e);

    if w < na::zero() || v + w > d {
        return None;
    }

    if t < na::zero() {
        Some((-t / d, -na::normalize(&n)))
    }
    else {
        Some((t / d, na::normalize(&n)))
    }
    // FIXME: it could be useful to return barycentric coordinates too
}

#[inline(always)]
pub fn segment_toi_and_normal_with_ray<N: Num + Ord + Signed + Algebraic,
                                       V: AlgebraicVecExt<N>>(
                                       a:   &V,
                                       b:   &V,
                                       ray: &Ray<V>)
                                       -> Option<(N, V)> {
    assert!(na::dim::<V>() == 2);

    /*
     * NOTE: redefine the cross product in order to not require the `Cross` trait.
     */
    #[inline(always)]
    fn perp<N: Num, V: VecExt<N>>(a: &V, b: &V) -> N {
        a.at(0) * b.at(1) - a.at(1) * b.at(0)
    }

    let ab = *b - *a;
    let d  = -ray.dir;

    let det = perp(&ab, &d);

    if det.is_zero() {
        return None;
    }

    let ap = ray.orig - *a;
    let t  = perp(&ap, &d);

    if (t < na::zero() && det < na::zero()) ||
       (t > na::zero() && det > na::zero()) {
        return None;
    }

    let det = det.abs();

    let w = perp(&ab, &ap);
    if w < na::zero() || w > det {
        return None;
    }

    let mut n = na::zero::<V>();
    n.set(0, ab.at(1));
    n.set(1, ab.at(0));

    if t < na::zero() {
        Some((-t / det, -na::normalize(&n)))
    }
    else {
        Some((t / det, na::normalize(&n)))
    }
}

impl<'a,
     N: Ord + Num + Float + Cast<f32> + Clone,
     V: Send + Freeze + AlgebraicVecExt<N> + Clone,
     M: Transform<V> + Rotate<V>>
RayCastWithTransform<N, V, M> for Subsimplex<'a, N, V> { }


impl<N: Num + Bounded + Orderable + Primitive + Algebraic + Float + Cast<f32>,
     V: Send + Freeze + AlgebraicVecExt<N> + Clone>
RayCast<N, V> for SubsimplexMesh<N, V> {
    // FIXME: find a way to refactore those two methods

    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        // FIXME: why cant the array type be infered here?
        let mut interferences: ~[uint] = ~[];

        {
            let mut visitor = RayInterferencesCollector::new(ray, &mut interferences);
            self.bvt().visit(&mut visitor);
        }

        // compute the minimum toi
        let mut toi: N = Bounded::max_value();

        for i in interferences.iter() {
            let subsimplex = self.subsimplex_at(*i);

            match subsimplex.toi_with_ray(ray) {
                None        => { },
                Some(ref t) => toi = toi.min(t)
            }
        }

        if toi == Bounded::max_value() {
            None
        }
        else {
            Some(toi)
        }
    }

    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        // FIXME: why cant the array type be infered here?
        let mut interferences: ~[uint] = ~[];

        {
            let mut visitor = RayInterferencesCollector::new(ray, &mut interferences);
            self.bvt().visit(&mut visitor);
        }

        // compute the minimum toi
        let mut toi: (N, V) = (Bounded::max_value(), na::zero());

        for i in interferences.iter() {
            let subsimplex = self.subsimplex_at(*i);

            match subsimplex.toi_and_normal_with_ray(ray) {
                None        => { },
                Some(t) => {
                    if *toi.first_ref() < *t.first_ref() {
                        toi = t
                    }
                }
            }
        }

        if *toi.first_ref() == Bounded::max_value() {
            None
        }
        else {
            Some(toi)
        }
    }
}


impl<N: Ord + Num + Float + Cast<f32> + Clone,
     V: AlgebraicVecExt<N> + Clone,
     M: Transform<V> + Rotate<V>>
RayCastWithTransform<N, V, M> for SubsimplexMesh<N, V> { }
