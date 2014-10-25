use na::{Pnt2, Transform, Rotate, Translate, Dim};
use na;
use ray::{Ray, LocalRayCast, RayCast, RayIntersection};
use geom::Ball;
use math::{Scalar, Point, Vect};



#[inline]
fn ball_uv<N, V>(normal: &V) -> Option<Pnt2<N>>
    where N: Scalar,
          V: Index<uint, N> + Dim {
    if na::dim::<V>() == 3 {
        let two_pi: N = Float::two_pi();
        let pi:     N = Float::pi();
        let _0_5:   N = na::cast(0.5f64);
        let uvx = _0_5 + (*normal)[2].atan2((*normal)[0]) / two_pi;
        let uvy = _0_5 - (*normal)[1].asin() / pi;

        Some(Pnt2::new(uvx, uvy))
    }
    else {
        None
    }
}

impl<N, P, V> LocalRayCast<N, P, V> for Ball<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<N> {
        ball_toi_with_ray(na::orig(), self.radius(), ray, solid).val1()
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        let (inside, inter) = ball_toi_with_ray(na::orig(), self.radius(), ray, solid);
        inter.map(|n| {
            let pos    = ray.orig + ray.dir * n;
            let normal = na::normalize(pos.as_vec());

            RayIntersection::new(n, if inside { -normal } else { normal })
        })
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        let (inside, inter) = ball_toi_with_ray(na::orig(), self.radius(), ray, solid);

        inter.map(|n| {
            let pos    = ray.orig + ray.dir * n;
            let normal = na::normalize(pos.as_vec());
            let uv     = ball_uv(&normal);

            RayIntersection::new_with_uvs(n, if inside { -normal } else { normal }, uv)
        })
    }
}

impl<N, P, V, M> RayCast<N, P, V, M> for Ball<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Translate<P> + Rotate<V> {
    #[inline]
    fn toi_with_transform_and_ray(&self, m: &M, ray: &Ray<P, V>, solid: bool) -> Option<N> {
        ball_toi_with_ray(m.translate(&na::orig()), self.radius(), ray, solid).val1()
    }
}

/// Computes the time of impact of a ray on a ball.
#[inline]
pub fn ball_toi_with_ray<N, P, V>(center: P, radius: N, ray: &Ray<P, V>, solid: bool) -> (bool, Option<N>)
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    let dcenter = ray.orig - center;

    let b = na::dot(&dcenter, &ray.dir);
    let c = na::sqnorm(&dcenter) - radius * radius;

    if c > na::zero() && b > na::zero() {
        (false, None)
    }
    else {
        let delta = b * b - c;

        if delta < na::zero() {
            // no solution
            (false, None)
        }
        else {
            let t = -b - delta.sqrt();

            if t <= na::zero() {
                // orig inside of the ball
                if solid {
                    (true, Some(na::zero()))
                }
                else {
                    (true, Some(-b + delta.sqrt()))
                }
            }
            else {
                (false, Some(t))
            }
        }
    }
}
