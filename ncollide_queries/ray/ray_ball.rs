use num::Float;
use na::{Pnt2, Rotate, Translate, BaseFloat};
use na;
use ray::{Ray, RayCast, RayIntersection};
use entities::shape::Ball;
use math::{Point, Vect};


#[inline]
fn ball_uv<V>(normal: &V) -> Option<Pnt2<V::Scalar>>
    where V: Vect {
    if na::dim::<V>() == 3 {
        let two_pi: V::Scalar = BaseFloat::two_pi();
        let pi:     V::Scalar = BaseFloat::pi();
        let _0_5:   V::Scalar = na::cast(0.5f64);
        let uvx = _0_5 + normal[2].atan2(normal[0]) / two_pi;
        let uvy = _0_5 - normal[1].asin() / pi;

        Some(Pnt2::new(uvx, uvy))
    }
    else {
        None
    }
}

impl<P, M> RayCast<P, M> for Ball<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Translate<P> + Rotate<P::Vect> {
    #[inline]
    fn toi_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<<P::Vect as Vect>::Scalar> {
        ball_toi_with_ray(&m.translate(&na::orig()), self.radius(), ray, solid).1
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        let center = m.translate(&na::orig());
        let (inside, inter) = ball_toi_with_ray(&center, self.radius(), ray, solid);

        inter.map(|n| {
            let pos    = ray.orig + ray.dir * n - center;
            let normal = na::normalize(&pos);

            RayIntersection::new(n, if inside { -normal } else { normal })
        })
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        let center = m.translate(&na::orig());
        let (inside, inter) = ball_toi_with_ray(&center, self.radius(), ray, solid);

        inter.map(|n| {
            let pos    = ray.orig + ray.dir * n - center;
            let normal = na::normalize(&pos);
            let uv     = ball_uv(&normal);

            RayIntersection::new_with_uvs(n, if inside { -normal } else { normal }, uv)
        })
    }
}

/// Computes the time of impact of a ray on a ball.
#[inline]
pub fn ball_toi_with_ray<P>(center: &P,
                            radius: <P::Vect as Vect>::Scalar,
                            ray:    &Ray<P>,
                            solid:  bool)
                            -> (bool, Option<<P::Vect as Vect>::Scalar>)
    where P: Point {
    let dcenter = ray.orig - *center;

    let a = na::sqnorm(&ray.dir);
    let b = na::dot(&dcenter, &ray.dir);
    let c = na::sqnorm(&dcenter) - radius * radius;

    if c > na::zero() && b > na::zero() {
        (false, None)
    }
    else {
        let delta = b * b - a * c;

        if delta < na::zero() {
            // no solution
            (false, None)
        }
        else {
            let t = (-b - delta.sqrt()) / a;

            if t <= na::zero() {
                // orig inside of the ball
                if solid {
                    (true, Some(na::zero()))
                }
                else {
                    (true, Some((-b + delta.sqrt()) / a))
                }
            }
            else {
                (false, Some(t))
            }
        }
    }
}
