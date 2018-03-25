use std::marker::PhantomData;
use std::f64;
use num::Zero;
use approx::ApproxEq;
use smallvec::SmallVec;

use na::{self, Real, Unit};
use utils;
use math::{Isometry, Point, Vector};

// XXX: we use P as the type parameter because without it
// we can't implement `transform_by` cleanly…
#[derive(Clone, Debug)]
pub struct PolyhedralCone<P: Point> {
    // FIXME: specialization, 2D polycones cannot contain more than 2 generators.
    generators: SmallVec<[Unit<P::Vector>; 4]>,
    phantom: PhantomData<P>,
}

impl<P: Point> PolyhedralCone<P> {
    pub fn new() -> Self {
        Self::from_slice(&[])
    }

    pub fn from_slice(normals: &[Unit<P::Vector>]) -> Self {
        PolyhedralCone {
            generators: SmallVec::from_slice(normals),
            phantom: PhantomData,
        }
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.generators.len()
    }

    #[inline]
    pub fn clear(&mut self) {
        self.generators.clear();
    }

    #[inline]
    pub fn generators(&self) -> &[Unit<P::Vector>] {
        &self.generators[..]
    }

    #[inline]
    pub fn add_generator(&mut self, gen: Unit<P::Vector>) {
        if na::dimension::<P::Vector>() == 2 && self.generators.len() == 2 {
            // FIXME: too restrictive?
            panic!("Polyhedral cone: 2D polyhedral cones cannot contain more than 2 generators.")
        }
        self.generators.push(gen);
    }

    pub fn transform_by<M: Isometry<P>>(&mut self, m: &M) {
        for gen in &mut self.generators {
            *gen = Unit::new_unchecked(m.transform_vector(gen.as_ref()));
        }
    }

    pub fn project(&self, dir: &Unit<P::Vector>) -> Unit<P::Vector> {
        if self.generators.len() == 0 {
            *dir
        } else if self.generators.len() == 1 {
            self.generators[0]
        } else {
            if na::dimension::<P::Vector>() == 2 {
                assert!(self.generators.len() == 2);
                let perp1 = utils::perp2(&**dir, &*self.generators[0]);
                let perp2 = utils::perp2(&**dir, &*self.generators[1]);
                let _0 = P::Real::zero();

                match (perp1 > _0, perp2 > _0) {
                    (true, true) => self.generators[0],
                    (false, false) => self.generators[1],
                    (false, true) => *dir,
                    (true, false) => {
                        if perp1 <= -perp2 {
                            self.generators[0]
                        } else {
                            self.generators[1]
                        }
                    }
                }
            } else {
                unimplemented!()
            }
        }
    }

    pub fn contains(&self, v: &P::Vector) -> bool {
        if let Some(dir) = Unit::try_new(*v, P::Real::default_epsilon()) {
            self.contains_dir(&dir)
        } else {
            true
        }
    }

    pub fn polar_contains_dir(&self, dir: &Unit<P::Vector>) -> bool {
        if self.generators.len() == 0 {
            // Empty polyhedral cone === the whole space.
            // Thus its polar contains only 0.
            false
        } else {
            for g in &self.generators {
                if na::dot(g.as_ref(), dir.as_ref()) > na::zero() {
                    return false;
                }
            }
            true
        }
    }

    pub fn contains_dir(&self, dir: &Unit<P::Vector>) -> bool {
        if self.generators.len() == 0 {
            true
        } else if na::dimension::<P::Vector>() == 2 {
            // NOTE: the following assumes the polycone
            // generator are ordered in CCW order.
            assert!(self.generators.len() == 2);
            let perp1 = utils::perp2(dir.as_ref(), &*self.generators[0]);
            let perp2 = utils::perp2(dir.as_ref(), &*self.generators[1]);
            let _0 = P::Real::zero();

            perp1 <= _0 && perp2 >= _0
        } else {
            // NOTE: the following does not makes any assumptions on the
            // polycone orientation.
            let mut sign = P::Real::zero();
            if self.generators.len() == 1 {
                // The polycone is degenerate and actually has only one generactor.
                let eps: P::Real = na::convert(f64::consts::PI / 180.0 * 0.1);
                let c_eps = eps.cos();
                let dot = na::dot(&*self.generators[0], dir.as_ref());
                dot >= c_eps
            } else if self.generators.len() == 2 {
                let eps = na::convert(f64::consts::PI / 180.0 * 0.1);
                let normal = utils::cross3(&*self.generators[1], &*self.generators[0]);

                if let Some(normal) = Unit::try_new(normal, na::zero()) {
                    if na::dot(&*normal, dir.as_ref()).abs() > eps {
                        return false;
                    }

                    let middle = (*self.generators[0] + *self.generators[1]) * na::convert(0.5);
                    if na::dot(&middle, dir.as_ref()) < na::zero() {
                        return false;
                    }

                    let cross1 = utils::cross3(&*self.generators[0], dir.as_ref());
                    let cross2 = utils::cross3(&*self.generators[1], dir.as_ref());

                    na::dot(&cross1, &*normal) * na::dot(&cross2, &*normal) <= na::zero()
                } else {
                    // FIXME: duplicate code with the case where we only have one generator.
                    // The polycone is degenerate and actually has only one generactor.
                    let c_eps = eps.cos();
                    let dot = na::dot(&*self.generators[0], dir.as_ref());
                    dot >= c_eps
                }
            } else {
                let mut sign = P::Real::zero();
                let mut center = P::Vector::zero();

                for i1 in 0..self.generators.len() {
                    let i2 = (i1 + 1) % self.generators.len();
                    let cross =
                        utils::cross3(self.generators[i1].as_ref(), self.generators[i2].as_ref());
                    let dot = na::dot(dir.as_ref(), &cross);
                    center += self.generators[i1].unwrap();

                    if sign.is_zero() {
                        sign = dot
                    } else if sign * dot < na::zero() {
                        return false;
                    }
                }

                // FIXME: is this a sufficient condition to determine if the
                // dir is not of the opposite cone?
                na::dot(&center, dir.as_ref()) >= na::zero()
            }
        }
    }
}
