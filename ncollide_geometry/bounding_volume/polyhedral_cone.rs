use std::marker::PhantomData;
use std::f64;
use num::{Zero, One};
use approx::ApproxEq;
use smallvec::SmallVec;

use na::{self, Real, Unit};
use utils;
use math::{Isometry, Point, Vector};

#[derive(Clone, Debug)]
pub enum PolyhedralCone<V: Vector> {
    Full,
    Empty,
    HalfLine(Unit<V>),
    HalfSpace(Unit<V>),
    OrthogonalSubspace(Unit<V>),
    Span(SmallVec<[Unit<V>; 4]>)
}

// FIXME: this hould be a method, be it can't because of the type parameter P.
pub fn transform_by<M: Isometry<P>, P: Point>(cone: &mut PolyhedralCone<P::Vector>, m: &M) {
    match *cone {
            PolyhedralCone::HalfLine(ref mut dir) => *dir = m.transform_unit_vector(dir),
            PolyhedralCone::HalfSpace(ref mut normal) => *normal = m.transform_unit_vector(normal),
            PolyhedralCone::OrthogonalSubspace(ref mut normal) => *normal = m.transform_unit_vector(normal),
            PolyhedralCone::Span(ref mut generators) => {
                for gen in generators {
                    *gen = m.transform_unit_vector(gen);
                }
            }
            PolyhedralCone::Full => {}
            PolyhedralCone::Empty => {}
    }
}

impl<V: Vector> PolyhedralCone<V> {
    pub fn unwrap_half_line(&self) -> Unit<V> {
        if let PolyhedralCone::HalfLine(dir) = *self {
            dir
        } else {
            panic!("This polyhedral cone is not a half-line.")
        }
    }

    // pub fn project(&self, dir: &Unit<V>) -> Unit<V> {
    //     if generators.len() == 0 {
    //         *dir
    //     } else if generators.len() == 1 {
    //         generators[0]
    //     } else {
    //         if na::dimension::<V>() == 2 {
    //             assert!(generators.len() == 2);
    //             let perp1 = utils::perp2(&**dir, &*generators[0]);
    //             let perp2 = utils::perp2(&**dir, &*generators[1]);
    //             let _0 = P::Real::zero();

    //             match (perp1 > _0, perp2 > _0) {
    //                 (true, true) => generators[0],
    //                 (false, false) => generators[1],
    //                 (false, true) => *dir,
    //                 (true, false) => {
    //                     if perp1 <= -perp2 {
    //                         generators[0]
    //                     } else {
    //                         generators[1]
    //                     }
    //                 }
    //             }
    //         } else {
    //             unimplemented!()
    //         }
    //     }
    // }

    pub fn contains(&self, v: &V) -> bool {
        if let Some(dir) = Unit::try_new(*v, V::Real::default_epsilon()) {
            self.contains_dir(&dir)
        } else {
            true
        }
    }

    pub fn polar_contains_dir(&self, dir: &Unit<V>) -> bool {
        let eps: V::Real = na::convert(f64::consts::PI / 180.0 * 0.01);
        let c_eps = eps.cos();
        
        match *self {
            PolyhedralCone::Full => false,
            PolyhedralCone::Empty => true,
            PolyhedralCone::HalfLine(ref generator) => na::dot(generator.as_ref(), dir.as_ref()) <= na::zero(),
            PolyhedralCone::HalfSpace(ref normal) => na::dot(normal.as_ref(), dir.as_ref()) >= na::zero(),
            PolyhedralCone::OrthogonalSubspace(ref normal) => na::dot(normal.as_ref(), dir.as_ref()).abs() >= V::Real::one() - c_eps,
            PolyhedralCone::Span(ref generators) => {
                for g in generators {
                    if na::dot(g.as_ref(), dir.as_ref()) > na::zero() {
                        return false;
                    }
                }
                true
            }
        }
    }

    pub fn contains_dir(&self, dir: &Unit<V>) -> bool {
        let eps: V::Real = na::convert(f64::consts::PI / 180.0 * 0.01);
        let c_eps = eps.cos();
        
        match *self {
            PolyhedralCone::Full => true,
            PolyhedralCone::Empty => false,
            PolyhedralCone::HalfLine(ref generator) => na::dot(generator.as_ref(), dir.as_ref()) >= V::Real::one() - c_eps,
            PolyhedralCone::HalfSpace(ref normal) => na::dot(normal.as_ref(), dir.as_ref()) <= na::zero(),
            PolyhedralCone::OrthogonalSubspace(ref normal) => na::dot(normal.as_ref(), dir.as_ref()).abs() <= c_eps,
            PolyhedralCone::Span(ref generators) => {
                if na::dimension::<V>() == 2 {
                    // NOTE: the following assumes the polycone
                    // generator are ordered in CCW order.
                    assert!(generators.len() == 2);
                    let perp1 = utils::perp2(dir.as_ref(), &*generators[0]);
                    let perp2 = utils::perp2(dir.as_ref(), &*generators[1]);
                    let _0 = V::Real::zero();

                    perp1 <= _0 && perp2 >= _0
                } else {
                    // NOTE: the following does not makes any assumptions on the
                    // polycone orientation.
                    let mut sign = V::Real::zero();
                    if generators.len() == 1 {
                        // The polycone is degenerate and actually has only one generactor.
                        let eps: V::Real = na::convert(f64::consts::PI / 180.0 * 0.1);
                        let c_eps = eps.cos();
                        let dot = na::dot(&*generators[0], dir.as_ref());
                        dot >= c_eps
                    } else if generators.len() == 2 {
                        let eps = na::convert(f64::consts::PI / 180.0 * 0.1);
                        let normal = utils::cross3(&*generators[1], &*generators[0]);

                        if let Some(normal) = Unit::try_new(normal, na::zero()) {
                            if na::dot(&*normal, dir.as_ref()).abs() > eps {
                                return false;
                            }

                            let middle = (*generators[0] + *generators[1]) * na::convert(0.5);
                            if na::dot(&middle, dir.as_ref()) < na::zero() {
                                return false;
                            }

                            let cross1 = utils::cross3(&*generators[0], dir.as_ref());
                            let cross2 = utils::cross3(&*generators[1], dir.as_ref());

                            na::dot(&cross1, &*normal) * na::dot(&cross2, &*normal) <= na::zero()
                        } else {
                            // FIXME: duplicate code with the case where we only have one generator.
                            // The polycone is degenerate and actually has only one generactor.
                            let c_eps = eps.cos();
                            let dot = na::dot(&*generators[0], dir.as_ref());
                            dot >= c_eps
                        }
                    } else {
                        let mut sign = V::Real::zero();
                        let mut center = V::zero();

                        for i1 in 0..generators.len() {
                            let i2 = (i1 + 1) % generators.len();
                            let cross =
                                utils::cross3(generators[i1].as_ref(), generators[i2].as_ref());
                            let dot = na::dot(dir.as_ref(), &cross);
                            center += generators[i1].unwrap();

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
    }
}
